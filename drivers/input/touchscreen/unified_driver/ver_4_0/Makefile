

obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_model_config_i2c.o
obj-y	+=  lgtp_model_config_misc.o
obj-y	+=  lgtp_platform_api_i2c.o
obj-y	+=  lgtp_platform_api_misc.o
obj-y	+=  lgtp_platform_api_power.o

ifeq ($(CONFIG_JDI_INCELL), y)
obj-y 	+= S3320/
endif

ifeq ($(CONFIG_MTK_TD4191),y)
obj-y	+=  td4191/
endif

ifeq ($(CONFIG_TOUCHSCREEN_FT6X36), y)
obj-y	+= FT6X36/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU202X), y)
obj-y 	+= LU202X/
endif

ifeq ($(CONFIG_TOUCHSCREEN_SN280H), y)
obj-y	+= SN280H/
endif

ifeq ($(CONFIG_MACH_MSM8939_M239DS_GLOBAL_COM), y)
obj-y 	+= MIT300/
endif

ifeq ($(CONFIG_MSM8909_M209), y)
obj-y   += MIT300/
endif

ifeq ($(CONFIG_TOUCHSCREEN_MELFAS_MIT300_PH1), y)
obj-y   += MIT300/
endif
