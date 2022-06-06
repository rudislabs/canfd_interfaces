std_msgs/Header header
uint8 priority					#valid [0,7] with 4 being nominal
bool is_annonymous				#false for regular transfer true for annonymous transfer
uint16 subject_id 				#valid [0, 8191]
uint8 source_node_id 			#valid [0,127]
uint8[63] data                  #63 bytes of data
uint8 crc						#crc 0xE0 + modulo 32 of sent message count
