# rpr0521-driver-proximity

http://rohmfs.rohm.com/en/products/databook/datasheet/opto/optical_sensor/opto_module/rpr-0521rs-e.pdf




&i2c_1 {
	rpr0521@38 {
		compatible = "yellowfin3,rpr0521";
		reg = <0x38>;
		#address-cells = <2>;
		#size-cells = <1>;
		interrupt-parent = <&msm_gpio>;
		interrupts = <93 0x2>;
		gpio-irq-pin = <&msm_gpio 93 0x02>;
	};
};





config RPR_0521RS
	tristate "ROHM RPR0521 ALS and proximity sensor driver"
	depends on I2C
	select REGMAP_I2C
	select IIO_TRIGGERED_BUFFER
	help
	  Say Y here if you want to build support for ROHM's RPR0521
	  ambient light and proximity sensor device.

	  To compile this driver as a module, choose M here:
	  the module will be called rpr0521.





obj-$(CONFIG_RPR_0521RS)	+= rpr0521.o



CONFIG_RPR_0521RS=y
