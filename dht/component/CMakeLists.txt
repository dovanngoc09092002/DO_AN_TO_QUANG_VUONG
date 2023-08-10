# idf_component_register(SRCS "dht.c"
#                     INCLUDE_DIRS ".")
set(app_src dht.c)
set(pre_req driver log)
idf_component_register(SRCS ${app_src}
                        INCLUDE_DIRS "."
                        REQUIRES ${pre_req})