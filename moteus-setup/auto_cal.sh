# sudo python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '1=1,2' --restore-cal moteus-cal/ri50_cal_1_leg.log
# sudo python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '1=1,2' --restore-cal moteus-cal/ri50_cal_2_leg.log
sudo python3 -m moteus.moteus_tool --target 1 --pi3hat-cfg '1=1,2' --restore-cal ri50_cal_1_leg.log
sudo python3 -m moteus.moteus_tool --target 2 --pi3hat-cfg '1=1,2' --restore-cal ri50_cal_2_leg.log