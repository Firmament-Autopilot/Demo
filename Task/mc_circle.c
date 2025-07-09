#include <firmament.h>

#include "FMS.h"
#include "INS.h"
#include "module/sysio/auto_cmd.h"
#include "module/sysio/gcs_cmd.h"
#include "module/task_manager/task_manager.h"

MCN_DECLARE(auto_cmd);
MCN_DECLARE(fms_output);

static fmt_err_t task_init(void)
{
    return FMT_EOK;
}

static void task_entry(void* parameter)
{
    McnNode_t fms_out_nod;
    FMS_Out_Bus fms_out;
    Auto_Cmd_Bus auto_cmd;

    fms_out_nod = mcn_subscribe(MCN_HUB(fms_output), NULL);

    /* Set mode to offboard */
    gcs_set_mode(PilotMode_Offboard);
    systime_mdelay(1000);

    gcs_set_cmd(FMS_Cmd_Takeoff, (float[7]) { 0 }); /* Send takeoff command */

    /* Wait until takeoff complete */
    while (1) {
        if (mcn_wait(fms_out_nod, RT_WAITING_FOREVER)) {
            mcn_copy(MCN_HUB(fms_output), fms_out_nod, &fms_out);
            if (fms_out.state == VehicleState_Hold) {
                break;
            }
        }
    }

    while (1) {
        auto_cmd.timestamp = systime_now_ms();
        auto_cmd.frame = FRAME_BODY_FRD;
        auto_cmd.u_cmd = 1.0;
        auto_cmd.psi_rate_cmd = PI / 6;
        auto_cmd.cmd_mask = U_CMD_VALID | PSI_RATE_CMD_VALID;

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