#ifndef SOBEL_FILTER_H_
#define SOBEL_FILTER_H_
#include <systemc>
#include <cmath>
#include <iomanip>
using namespace sc_core;

#include <tlm>
#include <tlm_utils/simple_target_socket.h>

#include "filter_def.h"

struct medfilter : public sc_module {
  tlm_utils::simple_target_socket<medfilter> tsock;

  sc_fifo<unsigned char> i_r;
  sc_fifo<unsigned char> i_g;
  sc_fifo<unsigned char> i_b;
  sc_fifo<unsigned char> en;
  sc_fifo<int> o_result;

  SC_HAS_PROCESS(medfilter);

  medfilter(sc_module_name n): 
    sc_module(n), 
    tsock("t_skt"), 
    base_offset(0) 
  {
    tsock.register_b_transport(this, &medfilter::blocking_transport);
    SC_THREAD(do_filter);
  }

  ~medfilter() {
	}
    sc_dt::sc_uint<8> med_buffer[(MASK_X*MASK_Y)]={0};
  	sc_dt::sc_uint<8> mean_buffer[MASK_X][MASK_Y]={{0,0,0},{0,0,0},{0,0,0}};
	  sc_dt::sc_uint<8> temp[(MASK_X*MASK_Y)]={0};
    sc_dt::sc_uint<8> grey;
    sc_dt::sc_uint<8>  en_med ;
     sc_dt::sc_uint<8>  en_mean ;
    unsigned int base_offset;



    void do_filter(){
    { wait(CLOCK_PERIOD, SC_NS); }
     
    while (true) {
       
          for (unsigned int v = 0; v<MASK_Y; ++v) {
              med_buffer[v]= (i_r.read() + i_g.read() + i_b.read()) / 3;
               en_med = en.read();
            }

            if(en_med==1){
                for(int i=0;i<9;i++){
 				          temp[i]=med_buffer[i];
			          }
              for(int i = 8; i > 0; i--){
    			      for(int j = 0; j <= i-1; j++){
        			    if( temp[j] > temp[j+1]){
           			    int tmp = temp[j];
            		    temp[j] = temp[j+1];
            		    temp[j+1] = tmp;
        			    }
                }
			        }
          
              o_result.write((int)temp[4]);
              }else{
              }


              for ( int v = 8; v >=3; --v) {            /*shift buffer*/
      		    med_buffer[v]=med_buffer[v-3];
   		 	      }	
          


		          int val_mean=0;

            for (unsigned int v = 0; v < MASK_Y; ++v) {
             mean_buffer[0][v]=(i_r.read() + i_g.read() + i_b.read()) / 3;
              en_mean = en.read();
             }

		          if(en_mean==1){	
			 	        for (unsigned int v = 0; v < MASK_Y; ++v) {
      				    for (unsigned int u = 0; u < MASK_X; ++u) {
    
        			    val_mean +=mean_buffer[u][v]*mask[u][v];
      				    }	
    			      }	                
              o_result.write(val_mean/10);

        	      }  


              for (unsigned int v = 0; v < MASK_Y; ++v) {
        	      for (unsigned int u = (MASK_X-1); u > 0; --u) {      //shift col1->col2 col0->col1
                  mean_buffer[u][v] = mean_buffer[u-1][v] ;
    	          }
              }


    }
  }







    void blocking_transport(tlm::tlm_generic_payload &payload, sc_core::sc_time &delay){
    wait(delay);
    // unsigned char *mask_ptr = payload.get_byte_enable_ptr();
    // auto len = payload.get_data_length();
    tlm::tlm_command cmd = payload.get_command();
    sc_dt::uint64 addr = payload.get_address();
    unsigned char *data_ptr = payload.get_data_ptr();

    addr -= base_offset;


    // cout << (int)data_ptr[0] << endl;
    // cout << (int)data_ptr[1] << endl;
    // cout << (int)data_ptr[2] << endl;
    word buffer;

    switch (cmd) {
      case tlm::TLM_READ_COMMAND:
        // cout << "READ" << endl;
        switch (addr) {
          case SOBEL_FILTER_RESULT_ADDR:
            buffer.uint = o_result.read();
            break;
          default:
            std::cerr << "READ Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
          }
        data_ptr[0] = buffer.uc[0];
        data_ptr[1] = buffer.uc[1];
        data_ptr[2] = buffer.uc[2];
        data_ptr[3] = buffer.uc[3];
        break;
      case tlm::TLM_WRITE_COMMAND:
        // cout << "WRITE" << endl;
        switch (addr) {
          case SOBEL_FILTER_R_ADDR:
            i_r.write(data_ptr[0]);
            i_g.write(data_ptr[1]);
            i_b.write(data_ptr[2]);
            en.write(data_ptr[3]);
            break;
          default:
            std::cerr << "WRITE Error! SobelFilter::blocking_transport: address 0x"
                      << std::setfill('0') << std::setw(8) << std::hex << addr
                      << std::dec << " is not valid" << std::endl;
        }
        break;
      case tlm::TLM_IGNORE_COMMAND:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      default:
        payload.set_response_status(tlm::TLM_GENERIC_ERROR_RESPONSE);
        return;
      }
      payload.set_response_status(tlm::TLM_OK_RESPONSE); // Always OK
  }
};
#endif

