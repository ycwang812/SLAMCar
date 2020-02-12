# 添加普通用户访问串口设备的规则，使普通用户可正常访问串口设备  
# 注意：此方法会让所有的普通用户都具有访问这些串口设备的权限，  
#       存在一定的安全隐患 
function add_rules()
{
	if [ $1 -eq 1 ]; then  
		su - root -c 'echo KERNEL==\"ttyUSB[0-9]*\",MODE=\"0666\" >> /etc/udev/rules.d/70-ttyusb.rules'  
	else  
		su - root -c 'echo KERNEL==\"ttyUSB[0-9]*\",MODE=\"0666\" > /etc/udev/rules.d/70-ttyusb.rules'  
	fi  
}  
function start_to_add() {  
	while : ;  
	do  
		add_rules $1  
		if [ $? -eq 0 ]; then  
			exit  
		fi  
	done  
}  
function ready_to_add() {  
	if [ -e /etc/udev/rules.d/70-ttyusb.rules ]; then  
		rules=`grep "KERNEL==.*ttyUSB*.*MODE=.*0666" /etc/udev/rules.d/70-ttyusb.rules`  
		if [ $rules ]; then  
			exit  
		fi  
		start_to_add 1  
	else  
		start_to_add 2  
	fi  
}  
ready_to_add
