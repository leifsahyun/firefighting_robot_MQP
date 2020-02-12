BUILD_ALL += $(OUT_DIR)/distance_detector_ros

$(OUT_DIR)/distance_detector_ros : \
					$(OUT_OBJ_DIR)/distance_detector_ros.o \
					libacconeer.a \
					libcustomer.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkfun.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LDLIBS) -o $@
