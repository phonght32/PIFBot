#ifndef _TB6560_H
#define _TB6560_H

#ifdef __cplusplus
extern "C" {
#endif

#include "../driver/include/timer.h"



typedef struct {

} tb6560_config_t;

int tb6560_init(tb6560_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* _TB6560_H */