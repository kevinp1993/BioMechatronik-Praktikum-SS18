#ifndef AMIRO_RWLOCK_H_
#define AMIRO_RWLOCK_H_

#include <ch.hpp>

namespace amiro {

/**
 * @brief An enhanced mutex with parallel read but exclusive write capabilities
 */
class RWLock
{
public:
  /**
   * @brief An enum to specify the favoritism of the RWLock
   */
  enum Favor : bool {
    FAVOR_READERS = false, /**< Favors readers, even if writers are waiting */
    FAVOR_WRITERS = true,  /**< Favors writers and holds new readers back */
  };

  /**
   * @brief An enum to specify what to wait for in the tryLockReading() and tryLockWriting() methods.
   */
  enum WaitingType : uint8_t {
    WAIT_FOR_NONE = 0,  /**< does not block at all */
    WAIT_FOR_MUTEX = 1, /**< blocks at the mutex */
    WAIT_FOR_QUEUE = 2, /**< waits for the queues */
  };

private:
  /**
   * @brief A status struct holding the current configuration of the RWLock.
   */
  struct AccessStatus {
    uint8_t reader_count : 7; /**< The number of currently active readers (128 max) */
    uint8_t favor        : 1; /**< The currently selected favoritism */
  };

  chibios_rt::Mutex mutex;             /**< A mutex lock used for synchronization */
  chibios_rt::CondVar waiting_readers; /**< A condition variable for waiting readers */
  chibios_rt::CondVar waiting_writers; /**< A condition variable for all waiting writers */
  AccessStatus status;                 /**< The current status of the RWLock */

public:
  /**
   * @brief Constructor of the RWLock class.
   * @param[in] favor The initial favoritism (default: FAVOR_WRITERS)
   */
  explicit RWLock(const Favor favor = FAVOR_WRITERS);

  /**
   * @brief Destructor of the RWLock class.
   */
  virtual ~RWLock();

  /**
   * @brief Lock the RWLock as a reader (shared mode)
   */
  void lockReading();

  /**
   * @brief Lock the RWLock as a writer (exclusive mode)
   */
  void lockWriting();

  /**
   * @brief Try to lock the RWLock as a reader (shared mode) but do not block
   * @param[in] wait Can be used to wait for the mutex to lock or the queue to be empty
   * @return True if the RWLock was successfully locked; False if not
   */
  bool tryLockReading(const WaitingType wait = WAIT_FOR_NONE);

  /**
   * @brief Try to lock the RWLock as a writer (exclusive mode) but do not block
   * @param[in] wait Can be used to wait for the mutex to lock or the queue to be empty
   * @return True if the RWLock was successfully locked; False if not
   */
  bool tryLockWriting(const WaitingType wait = WAIT_FOR_NONE);

  /**
   * @brief Unlocks the RWLock.
   * If it was locked in shared or exclusive mode is detected automatically.
   */
  void unlock();

  /**
   * @brief Set the favoritism of the RWLock
   * @param[in] favor Either FAVOR_READERS or FAVOR_WRITERS
   */
  void setFavoritism(const Favor favor);

  /**
   * @brief Try to set the favoritism of the RWLock but do not block
   * @param[in] favor Either FAVOR_READERS or FAVOR_WRITERS
   * @return True if the favoritism was successfully set; False if not
   */
  bool trySetFavoritism(const Favor favor);

};

} // end of namespace amiro

#endif // AMIRO_RWLOCK_H_

