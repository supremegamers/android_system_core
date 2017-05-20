/*
 * Copyright (C) 2012 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "libsuspend"
//#define LOG_NDEBUG 0

#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <android-base/file.h>
#include <android-base/logging.h>
#include <android-base/strings.h>
#include <android-base/properties.h>
#include <android-base/stringprintf.h>

#include <linux/uinput.h>
#include <dirent.h>
#include <poll.h>

#include "autosuspend_ops.h"

#define BASE_SLEEP_TIME 100000
#define MAX_SLEEP_TIME 60000000
#define MAX_POWERBTNS 3

static constexpr char default_sleep_state[] = "mem";
static constexpr char fallback_sleep_state[] = "freeze";

static int state_fd = -1;
static int uinput_fd = -1;
static int wakeup_count_fd;

using android::base::GetProperty;
using android::base::ReadFdToString;
using android::base::StringPrintf;
using android::base::Trim;
using android::base::WriteStringToFd;

static pthread_t suspend_thread;
static sem_t suspend_lockout;
static void (*wakeup_func)(bool success) = NULL;
static int sleep_time = BASE_SLEEP_TIME;
static constexpr char sys_power_state[] = "/sys/power/state";
static constexpr char sys_power_wakeup_count[] = "/sys/power/wakeup_count";
static bool autosuspend_is_init = false;

static void update_sleep_time(bool success) {
    if (success) {
        sleep_time = BASE_SLEEP_TIME;
        return;
    }
    // double sleep time after each failure up to one minute
    sleep_time = MIN(sleep_time * 2, MAX_SLEEP_TIME);
}

static void emit_key(int ufd, int key_code, int val)
{
    struct input_event iev;
    iev.type = EV_KEY;
    iev.code = key_code;
    iev.value = val;
    iev.time.tv_sec = 0;
    iev.time.tv_usec = 0;
    write(ufd, &iev, sizeof(iev));
    iev.type = EV_SYN;
    iev.code = SYN_REPORT;
    iev.value = 0;
    write(ufd, &iev, sizeof(iev));
    LOG(INFO) << StringPrintf("send key %d (%d) on fd %d", key_code, val, ufd);
}

static void send_key_wakeup(int ufd)
{
    emit_key(ufd, KEY_WAKEUP, 1);
    emit_key(ufd, KEY_WAKEUP, 0);
}

static void send_key_power(int ufd, bool longpress)
{
    emit_key(ufd, KEY_POWER, 1);
    if (longpress) sleep(2);
    emit_key(ufd, KEY_POWER, 0);
}

static int openfds(struct pollfd pfds[])
{
    int cnt = 0;
    const char *dirname = "/dev/input";
    struct dirent *de;
    DIR *dir;

    if ((dir = opendir(dirname))) {
        while ((cnt < MAX_POWERBTNS) && (de = readdir(dir))) {
            int fd;
            char name[PATH_MAX];
            if (de->d_name[0] != 'e') /* eventX */
                continue;
            snprintf(name, PATH_MAX, "%s/%s", dirname, de->d_name);
            fd = open(name, O_RDWR | O_NONBLOCK);
            if (fd < 0) {
                LOG(ERROR) << StringPrintf("could not open %s, %s", name, strerror(errno));
                continue;
            }
            name[sizeof(name) - 1] = '\0';
            if (ioctl(fd, EVIOCGNAME(sizeof(name) - 1), &name) < 1) {
                LOG(ERROR) << StringPrintf("could not get device name for %s, %s", name, strerror(errno));
                name[0] = '\0';
            }
            // TODO: parse /etc/excluded-input-devices.xml
            if (strcmp(name, "Power Button")) {
                close(fd);
                continue;
            }

            LOG(INFO) << StringPrintf("open %s(%s) ok fd=%d", de->d_name, name, fd);
            pfds[cnt].events = POLLIN;
            pfds[cnt++].fd = fd;
        }
        closedir(dir);
    }

    return cnt;
}

static void *powerbtnd_thread_func(void *arg __attribute__((unused)))
{
    int cnt, timeout, pollres;
    bool longpress = true;
    bool doubleclick = android::base::GetBoolProperty("poweroff.doubleclick", false);
    struct pollfd pfds[MAX_POWERBTNS];

    timeout = -1;
    cnt = openfds(pfds);

    while (cnt > 0) {
        if ((pollres = poll(pfds, cnt, timeout)) < 0) {
            LOG(ERROR) << "poll error: " << strerror(errno);
            break;
        }
        LOG(VERBOSE) << "pollres=" << pollres << " timeout=" << timeout;
        if (pollres == 0) {
            LOG(INFO) << "timeout, send one power key";
            send_key_power(uinput_fd, 0);
            timeout = -1;
            longpress = true;
            continue;
        }
        for (int i = 0; i < cnt; ++i) {
            if (pfds[i].revents & POLLIN) {
                struct input_event iev;
                size_t res = read(pfds[i].fd, &iev, sizeof(iev));
                if (res < sizeof(iev)) {
                    LOG(WARNING) << StringPrintf("insufficient input data(%zd)? fd=%d", res, pfds[i].fd);
                    continue;
                }
                LOG(DEBUG) << StringPrintf("type=%d code=%d value=%d from fd=%d", iev.type, iev.code, iev.value, pfds[i].fd);
                if (iev.type == EV_KEY && iev.code == KEY_POWER && !iev.value) {
                    if (!doubleclick || timeout > 0) {
                        send_key_power(uinput_fd, longpress);
                        timeout = -1;
                    } else {
                        timeout = 1000; // one second
                    }
                } else if (iev.type == EV_SYN && iev.code == SYN_REPORT && iev.value) {
                    LOG(INFO) << "got a resuming event";
                    longpress = false;
                    timeout = 1000; // one second
                }
            }
        }
    }

    return NULL;
}

static void init_android_power_button()
{
    static pthread_t powerbtnd_thread;
    struct uinput_user_dev ud;

    if (uinput_fd >= 0) return;

    uinput_fd = open("/dev/uinput", O_WRONLY | O_NDELAY);
    if (uinput_fd < 0) {
        LOG(ERROR) << "could not open uinput device: " << strerror(errno);
        return;
    }

    memset(&ud, 0, sizeof(ud));
    strcpy(ud.name, "Android Power Button");
    write(uinput_fd, &ud, sizeof(ud));
    ioctl(uinput_fd, UI_SET_EVBIT, EV_KEY);
    ioctl(uinput_fd, UI_SET_KEYBIT, KEY_POWER);
    ioctl(uinput_fd, UI_SET_KEYBIT, KEY_WAKEUP);
    ioctl(uinput_fd, UI_DEV_CREATE, 0);

    pthread_create(&powerbtnd_thread, NULL, powerbtnd_thread_func, NULL);
    pthread_setname_np(powerbtnd_thread, "powerbtnd");
}

static bool sleep_state_available(const char *state)
{
    std::string buf;
    if (state_fd < 0 || !ReadFdToString(state_fd, &buf)) {
        PLOG(ERROR) << "Error reading from " << sys_power_state;
        return false;
    }
    return buf.find(state) != std::string::npos;
}

static const std::string &get_sleep_state()
{
    static std::string sleep_state;

    if (sleep_state.empty()) {
        sleep_state = GetProperty("sleep.state", "");
        if (!sleep_state.empty()) {
            LOG(INFO) << "autosuspend using sleep.state property (" << sleep_state << ")";
        } else if (sleep_state_available(default_sleep_state)) {
            sleep_state = default_sleep_state;
            LOG(INFO) << "autosuspend using default sleep_state (" << sleep_state << ")";
        } else {
            sleep_state = fallback_sleep_state;
            LOG(WARNING) << "autosuspend '" << default_sleep_state << "' unavailable, using fallback state (" << sleep_state << ")";
        }
    }
    return sleep_state;
}

static void* suspend_thread_func(void* arg __attribute__((unused))) {
    bool success = true;

    while (true) {
        update_sleep_time(success);
        usleep(sleep_time);
        success = false;
        LOG(VERBOSE) << "read wakeup_count";
        lseek(wakeup_count_fd, 0, SEEK_SET);
        std::string wakeup_count;
        if (!ReadFdToString(wakeup_count_fd, &wakeup_count)) {
            PLOG(ERROR) << "error reading from " << sys_power_wakeup_count;
            continue;
        }

        wakeup_count = Trim(wakeup_count);
        if (wakeup_count.empty()) {
            LOG(ERROR) << "empty wakeup count";
            continue;
        }

        LOG(VERBOSE) << "wait";
        int ret = sem_wait(&suspend_lockout);
        if (ret < 0) {
            PLOG(ERROR) << "error waiting on semaphore";
            continue;
        }

        LOG(VERBOSE) << "write " << wakeup_count << " to wakeup_count";
        auto sleep_state = get_sleep_state();
        if (WriteStringToFd(wakeup_count, wakeup_count_fd)) {
            LOG(VERBOSE) << "write " << sleep_state << " to " << sys_power_state;
            success = WriteStringToFd(sleep_state, state_fd);
            if (success) {
                send_key_wakeup(uinput_fd);
            }

            void (*func)(bool success) = wakeup_func;
            if (func != NULL) {
                (*func)(success);
            }
        } else {
            PLOG(ERROR) << "error writing to " << sys_power_wakeup_count;
        }

        LOG(VERBOSE) << "release sem";
        ret = sem_post(&suspend_lockout);
        if (ret < 0) {
            PLOG(ERROR) << "error releasing semaphore";
        }
    }
    return NULL;
}

static int init_state_fd(void) {
    if (state_fd >= 0) {
        return 0;
    }

    int fd = TEMP_FAILURE_RETRY(open(sys_power_state, O_CLOEXEC | O_RDWR));
    if (fd < 0) {
        PLOG(ERROR) << "error opening " << sys_power_state;
        return -1;
    }

    state_fd = fd;
    LOG(INFO) << "init_state_fd success";
    return 0;
}

static int autosuspend_init(void) {
    if (autosuspend_is_init) {
        return 0;
    }

    int ret = init_state_fd();
    if (ret < 0) {
        return -1;
    }

    wakeup_count_fd = TEMP_FAILURE_RETRY(open(sys_power_wakeup_count, O_CLOEXEC | O_RDWR));
    if (wakeup_count_fd < 0) {
        PLOG(ERROR) << "error opening " << sys_power_wakeup_count;
        goto err_open_wakeup_count;
    }

    ret = sem_init(&suspend_lockout, 0, 0);
    if (ret < 0) {
        PLOG(ERROR) << "error creating suspend_lockout semaphore";
        goto err_sem_init;
    }

    ret = pthread_create(&suspend_thread, NULL, suspend_thread_func, NULL);
    if (ret) {
        LOG(ERROR) << "error creating thread: " << strerror(ret);
        goto err_pthread_create;
    }

    LOG(VERBOSE) << "autosuspend_init success";
    autosuspend_is_init = true;
    return 0;

err_pthread_create:
    sem_destroy(&suspend_lockout);
err_sem_init:
    close(wakeup_count_fd);
err_open_wakeup_count:
    return -1;
}

static int autosuspend_wakeup_count_enable(void) {
    LOG(VERBOSE) << "autosuspend_wakeup_count_enable";

    int ret = autosuspend_init();
    if (ret < 0) {
        LOG(ERROR) << "autosuspend_init failed";
        return ret;
    }

    ret = sem_post(&suspend_lockout);
    if (ret < 0) {
        PLOG(ERROR) << "error changing semaphore";
    }

    LOG(VERBOSE) << "autosuspend_wakeup_count_enable done";

    return ret;
}

static int autosuspend_wakeup_count_disable(void) {
    LOG(VERBOSE) << "autosuspend_wakeup_count_disable";

    if (!autosuspend_is_init) {
        return 0;  // always successful if no thread is running yet
    }

    int ret = sem_wait(&suspend_lockout);

    if (ret < 0) {
        PLOG(ERROR) << "error changing semaphore";
    }

    LOG(VERBOSE) << "autosuspend_wakeup_count_disable done";

    return ret;
}

static int force_suspend(int timeout_ms) {
    LOG(VERBOSE) << "force_suspend called with timeout: " << timeout_ms;

    int ret = init_state_fd();
    if (ret < 0) {
        return ret;
    }

    return WriteStringToFd(get_sleep_state(), state_fd) ? 0 : -1;
}

static void autosuspend_set_wakeup_callback(void (*func)(bool success)) {
    if (wakeup_func != NULL) {
        LOG(ERROR) << "duplicate wakeup callback applied, keeping original";
        return;
    }
    wakeup_func = func;
}

struct autosuspend_ops autosuspend_wakeup_count_ops = {
    .enable = autosuspend_wakeup_count_enable,
    .disable = autosuspend_wakeup_count_disable,
    .force_suspend = force_suspend,
    .set_wakeup_callback = autosuspend_set_wakeup_callback,
};

struct autosuspend_ops* autosuspend_wakeup_count_init(void) {
    init_android_power_button();
    return &autosuspend_wakeup_count_ops;
}
