#include <fbs/trace.h>
#include <stdio.h>
int main(void) {
    fbs_trace_config config = fbs_trace_config_default();
    fbs_trace_context *context = NULL;
    if (fbs_trace_create(&config, NULL, &context) != 0) return 1;
    printf("API version: %u\n", fbs_trace_version());
    fbs_trace_destroy(context);
    return 0;
}
