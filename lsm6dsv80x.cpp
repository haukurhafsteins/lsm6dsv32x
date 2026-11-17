// Compatibility translation unit: retained for existing superproject source lists.
// The selected part is controlled by IMU_LSM6DSV_PART (default 80).
#include "detail/driver_impl.hpp"
#include "detail/public_api.hpp"
#if IMU_LSM6DSV_PART == 80
#include "detail/legacy80.hpp"
#endif
