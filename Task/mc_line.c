#include <firmament.h>

#include "FMS.h"
#include "INS.h"
#include "module/sysio/auto_cmd.h"
#include "module/sysio/gcs_cmd.h"
#include "module/task_manager/task_manager.h"

MCN_DECLARE(auto_cmd);
MCN_DECLARE(fms_output);
MCN_DECLARE(ins_output);

static fmt_err_t task_init(void)
{
    return FMT_EOK;
}

static void task_entry(void* parameter)
{
    float A[2], B[2];
    McnNode_t fms_out_nod;
    McnNode_t ins_out_nod;
    FMS_Out_Bus fms_out;
    INS_Out_Bus ins_out;
    Auto_Cmd_Bus auto_cmd;

    fms_out_nod = mcn_subscribe(MCN_HUB(fms_output), NULL);
    ins_out_nod = mcn_subscribe(MCN_HUB(ins_output), NULL);

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

    systime_mdelay(2000);

    if (mcn_wait(ins_out_nod, RT_WAITING_FOREVER)) {
        mcn_copy(MCN_HUB(ins_output), ins_out_nod, &ins_out);
    }
    /* start point */
    A[0] = ins_out.x_R;
    A[1] = ins_out.y_R;
    /* end point */
    B[0] = 10;
    B[1] = 5;

    float alpha = atan(B[1] / B[0]);
    float dx = 0.2 * cosf(alpha);
    float dy = 0.2 * sinf(alpha);

    while (1) {
        A[0] += dx;
        A[1] += dy;

        auto_cmd.timestamp = systime_now_ms();
        auto_cmd.frame = FRAME_GLOBAL_NED;
        auto_cmd.x_cmd = A[0];
        auto_cmd.y_cmd = A[1];
        auto_cmd.cmd_mask = X_CMD_VALID | Y_CMD_VALID;

        mcn_publish(MCN_HUB(auto_cmd), &auto_cmd);

        sys_msleep(100);

        if (A[0] >= B[0]) {
            /* reach point */
            break;
        }
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