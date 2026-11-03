#include "config.hpp"
#include "launcher.hpp"
#include "motors.hpp"
#include "remoter_receiver.hpp"
#include "user_config.hpp"
#include "sysctrl.hpp"
#include "trigger.hpp"
#include "main.h"
#include "tx_api.h"

namespace
{
TX_THREAD thread;
alignas(8) uint8_t stack[dart::cfg::task::stack_size];
char name[] = "dart";
memory::buffer<64> dr16_dma RAM_D1_BSS; //将dr16的buffer放到RAMD1
remotes::dr16::receiver dr16;

void run(ULONG)
{
    dart::sysctrl sysctrl;
    dart::launcher launcher;
    for (;;)
    {
        const uint32_t now = tx_time_get();
        remotes::reading dr16_input{};
        const bool dr16_online =
            dr16.read(dr16_input, now, user_config::remoter::offline_timeout_ms);
        auto feedback = dart::motors::read(now);
        feedback.trigger_done = dart::trigger::done();
        const auto command = sysctrl.update(dr16_input, dr16_online, launcher.state());
        const auto output = launcher.update(command, feedback, now);
        dart::motors::write(output, now);
        dart::trigger::update(output.trigger_release, now);
        tx_thread_sleep(dart::cfg::task::period_ticks);
    }
}
} // namespace

extern "C" void app_start()
{
    if (user_config::remoter_dr16_enabled)
    {
        (void)dr16.start(user_config::dr16_uart, user_config::dr16_line, dr16_dma);
    }
    if (user_config::remoter_vt03_enabled)
    {
        (void)vt03.start(user_config::vt03_uart, user_config::vt03_line, vt03_dma);
    }
    (void)dart::motors::start();
    (void)dart::trigger::start();
    (void)tx_thread_create(&thread, name, run, 0, stack, sizeof(stack),
        dart::cfg::task::thread_priority, dart::cfg::task::thread_priority,
        TX_NO_TIME_SLICE, TX_AUTO_START);
}
