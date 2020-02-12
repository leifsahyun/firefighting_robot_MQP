BUILD_ALL += $(OUT_DIR)/ref_app_rf_certification_test_rpi_sparkfun_a111_r2c

$(OUT_DIR)/ref_app_rf_certification_test_rpi_sparkfun_a111_r2c : \
					libacc_rf_certification_test.a \
					libacconeer.a \
					libcustomer.a \
					$(OUT_OBJ_DIR)/acc_board_rpi_sparkfun.o
	@echo "    Linking $(notdir $@)"
	$(SUPPRESS)$(LINK.o) -Wl,--start-group $^ -Wl,--end-group $(LDLIBS) -o $@
