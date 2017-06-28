#ifndef CELL_H_
#define CELL_H_

#include <inttypes.h>

namespace perception {

// TODO: remove CellBase
template <typename T>
class CellBase {
 public:
  CellBase<T>() : size_(sizeof(T)) {

  }

  ~CellBase() {}

  virtual void clear() {}

  CellBase(const CellBase& other) : size_(sizeof(T)) {}

 private:
  uint32_t size_;
//  uint32_t const size_; // No assigment operator possible :-(
};


} // namespace perception

#endif
