std_msgs/Header header
uint8 priority					#[0,7] with 4 being nominal
bool request_not_response 		#false if response true if request 
uint16 service_id 				#[0,511]
uint8 destination_node_id 		#valid [0,127]
uint8 source_node_id 			#valid [0,127]
uint8[63] data                  #63 bytes of data
uint8 crc						#crc 0xE0 + modulo 32 of sent message count
