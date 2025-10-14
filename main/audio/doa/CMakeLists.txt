idf_component_register(
    SRCS "doa_dual_mic.c" "kiss_fft.c" "kiss_fftr.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos esp_log
)
