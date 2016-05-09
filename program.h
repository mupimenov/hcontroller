#ifndef PROGRAM_H_
#define PROGRAM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void program_init(void);
void program_reset(void);
void program_execute(void);

#ifdef __cplusplus
}
#endif

#endif /* PROGRAM_H_ */
