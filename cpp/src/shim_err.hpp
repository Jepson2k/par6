/* Internal helper shared by the par6_shim translation units. */
#ifndef PAR6_SHIM_ERR_HPP
#define PAR6_SHIM_ERR_HPP

#include <cstdint>
#include <cstdio>

namespace par6_shim_detail {

inline void write_err(char *err_buf, int32_t err_len, const char *msg) {
    if (err_buf != nullptr && err_len > 0) {
        std::snprintf(err_buf, static_cast<size_t>(err_len), "%s", msg);
    }
}

} // namespace par6_shim_detail

#endif /* PAR6_SHIM_ERR_HPP */
