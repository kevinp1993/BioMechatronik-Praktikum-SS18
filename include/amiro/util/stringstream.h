#ifndef STRINGSTREAM_H_
#define STRINGSTREAM_H_

#ifdef __cplusplus
extern "C" {
#endif

struct BaseSequentialStreamVMT;

struct stringstream_t {
  const struct BaseSequentialStreamVMT *vmt;
  uint8_t string[256];
  uint8_t ins_ix;
  uint8_t out_ix;
};

void StringStreamInit(void *instance);
size_t StringStreamWrite(void *instance, const uint8_t *bp, size_t n);
size_t StringStreamRead(void *instance, uint8_t *bp, size_t n);
msg_t StringStreamPut(void *instance, uint8_t b);
msg_t StringStreamGet(void *instance);
size_t StringStreamTellp(void *instance);
uint8_t *StringStreamStrGet(void *instance);
void StringStreamStrSet(void *instance, const char *str);

#ifdef __cplusplus
}
#endif

#endif /* STRINGSTREAM_H_ */
