# rpr0521-driver-proximity

[Datasheet RPR0521-RS](http://rohmfs.rohm.com/en/products/databook/datasheet/opto/optical_sensor/opto_module/rpr-0521rs-e.pdf)

[Datasheet STK3X1X](http://pro0fc108.hkpic1.websiteonline.cn/upload/hc7r.pdf)

<b>Direction:</b> {AOSP}/kernel/driver/iio/light/

1. Device tree

```
&i2c_1 {
	rpr@38 {
		compatible = "yellowfin3,rpr0521";
		reg = <0x38>;
		#address-cells = <2>;
		#size-cells = <1>;
		interrupt-parent = <&msm_gpio>;
		interrupts = <93 0x2>;
		vdd-supply = <&pm8909_l17>;
	  	vio-supply = <&pm8909_l6>;
		rpr,irq-gpio = <&msm_gpio 94 0x02>;
		rpr,transmittance = <1000>;
		rpr,state-reg = <0x08>;
		rpr,psctrl-reg = <0x20>;
		rpr,alsctrl-reg = <0x02>;
		rpr,ps-thdh = <4096>;
		rpr,ps-thdl = <2800>;
		rpr,ps-delta-time = <10>;
		rpr,ps-offset = <10>;
		// rpr,use-fir;
	};
};

```

2. Kconfig

```
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
```

3. Makefile

```
obj-$(CONFIG_RPR_0521RS)	+= rpr0521.o
```

4. Defconfig

```
# 9. proximity sensor RPR_0521RS
CONFIG_IIO=y
CONFIG_IIO_TRIGGER=y
CONFIG_IIO_BUFFER=y
CONFIG_IIO_TRIGGERED_BUFFER=y
CONFIG_RPR_0521RS=y
```
