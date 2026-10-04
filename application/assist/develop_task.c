// 开发新功能时可以使用本任务进行功能测试

#include "develop_task.h"

#include "FreeRTOS.h"
#include "task.h"

void develop_task(void const * pvParameters)
{
    (void)pvParameters;

    // 空闲一段时间
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        // 在此添加临时功能测试代码

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
