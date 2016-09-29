#ifdef __cplusplus
#include <cstdint>
extern "C" {
#else
#include <stdint.h>
#endif

uint32_t read_header(unsigned char* buf);
uint32_t read_body(int csock, uint32_t siz, char* fmt_buffer);

#ifdef __cplusplus
}
#endif
