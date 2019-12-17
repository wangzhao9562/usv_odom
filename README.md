Ros pacakge which implements communication between main controller and base controller of ARMs USV

ARMs USV communication protocol
Base controller to main controller:  
0xa5 0x5a ship_num latitude_bit1 latitude_bit2 latitude_bit3 latitude_bit4 longitude_bit1 longitude_bit2 longitude_bit3 longitude_bit4 velocity_bit1 velocity_bit2 origin_lat_bit1 origin_lat_bit2 origin_lat_bit3 origin_lat_bit4 origin_lng_bit1 origin_lng_bit2 origin_lng_bit3 origin_lng_bit4 reserve_bit1 reserve_bit2 pack_len 0xaa

Main controller to base controller:  
1. Open Control:
0xa1 0x1a pack_len ship_num function_bit rud_bit speed_bit 0xaa  
2. Point follow:  
0xa1 0x1a pack_len ship_num function_bit target_lat_hbit target_lat_bit2 target_lat_bit3 target_lat_lbit target_lng_hbit target_lng_bit2 target_lng_bit3 target_lng_lbit 0xaa
3. PID settting:
0xa1 0x1a pack_len ship_num function_bit kp_bit ki_bit kd_bit kp1_bit ki1_bit kd1_bit 0xaa
4. Origin setting:
0xa1 0x1a pack_len ship_num function_bit 0xaa

