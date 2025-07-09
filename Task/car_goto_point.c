#include <firmament.h>

#include "FMS.h"
#include "INS.h"
#include "module/sysio/auto_cmd.h"
#include "module/sysio/gcs_cmd.h"
#include "module/task_manager/task_manager.h"

MCN_DECLARE(auto_cmd);

static fmt_err_t task_init(void)
{
    return FMT_EOK;
}

static void task_entry(void* parameter)
{
    Auto_Cmd_Bus auto_cmd;

    /* Set mode to offboard */
    gcs_set_mode(PilotMode_Offboard);

    systime_mdelay(1000);

    /* Arm the vehicle */
    gcs_set_cmd(FMS_Cmd_PreArm, (float[7]) { 0 }); /* Send takeoff command */

    systime_mdelay(1000);

    while (1) {
        auto_cmd.timestamp = systime_now_ms();
        auto_cmd.frame = FRAME_LOCAL_FRD; // FRAME_GLOBAL_NED | FRAME_LOCAL_FRD | FRAME_BODY_FRD
        auto_cmd.x_cmd = -10;
        auto_cmd.y_cmd = 10;
        auto_cmd.u_cmd = 2.0;
        auto_cmd.cmd_mask = X_CMD_VALID
            | Y_CMD_VALID
            | U_CMD_VALID;

        mcn_publish(MCN_HUB(auto_cmd), &auto_cmd);

        sys_msleep(100);
    }
}

TASK_EXPORT __fmt_task_desc = {
    .name = "offboard",
    .init = task_init,
    .entry = task_entry,
    .priority = 25,
    .auto_start = false,
    .stack_size = 4096,
    .param = NULL,
    .dependency = NULL
};