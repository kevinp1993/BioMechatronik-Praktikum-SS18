#ifndef AMIRO_USERTHREAD_H_
#define AMIRO_USERTHREAD_H_

#include <ch.hpp>
#include <amiroosconf.h>

namespace amiro {

class UserThread : public chibios_rt::BaseStaticThread<USER_THREAD_STACK_SIZE>
{
public:

  enum State {
    IDLE,
    OBSTACLE_AVOIDANCE,
    WII_STEERING,
  };

public:
  explicit UserThread();

  virtual ~UserThread();

  virtual msg_t main();

  void setNextState(const State state);

  State getCurrenState() const;

  msg_t setWiiAddress(const char* address);

  float setWiiDeadzone(const float deadzone);
};

} // end of namespace amiro

#endif // AMIRO_USERTHREAD_H_
