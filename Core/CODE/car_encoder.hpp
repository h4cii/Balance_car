#ifndef CAR_ENCODER_HPP
#define CAR_ENCODER_HPP

#include <stdint.h>

namespace car {

struct EncoderCounts {
    int16_t left_counts;
    int16_t right_counts;
};

class EncoderReader final {
public:
    void init();
    EncoderCounts read();
};

extern EncoderReader encoder;

}  // namespace car

#endif
