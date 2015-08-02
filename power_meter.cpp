#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <complex>
#include <cmath>

#define windows_count 1000

namespace po = boost::program_options;

// prototypes
double measure_rx_power( std::vector<std::complex<float> > &buff );
double windows_to_power_dBm( std::vector<double> &pm );

// real main is a try/catch handler wrapping this function call.
// UHD software uses exception throwing
int UHD_SAFE_MAIN( int argc, char *argv[] ) {
  uhd::set_thread_priority_safe();

  //variables to be set by program options
  std::string args, file, ant, subdev, ref;
  size_t total_num_samps;
  double rate, freq, gain, bw;
  std::string addr;

  //setup the program options
  po::options_description desc("Allowed options");
  desc.add_options()
    ("help", "help message")
    ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
    ("nsamps", po::value<size_t>(&total_num_samps)->default_value(1000), "total number of samples to receive")
    ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
    ("freq", po::value<double>(&freq)->default_value(0), "rf center frequency in Hz")
    ("gain", po::value<double>(&gain)->default_value(0), "gain for the RF chain")
    ("ant", po::value<std::string>(&ant), "antenna selection")
    ("subdev", po::value<std::string>(&subdev), "subdevice specification")
    ("bw", po::value<double>(&bw), "analog frontend filter bandwidth in Hz")
    ("addr", po::value<std::string>(&addr)->default_value("192.168.1.10"), "resolvable server address")
    ("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
    ("int-n", "tune USRP with integer-N tuning")
  ;
 
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  //print the help message if asked for, or needed
  if (vm.count("help")){
    std::cout << boost::format("UHD RX to UDP %s") % desc << std::endl;
    return ~0;
  }

  // create a usrp device
  std::cout << std::endl;
  std::cout << boost::format("Creatin the usrp device with: %s...") % args << std::endl;
  uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
  std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

  // lock clocks
  usrp->set_clock_source(ref);

  // set the rx sample rate
  std::cout << boost::format("Requesting Sample Rate: %f Msps...") % (rate/1e6) << std::endl;
  usrp->set_rx_rate(rate);
  std::cout << boost::format("Sample Rate Acquired: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl;

  // set the RX center frequency
  std::cout << boost::format("Requesting RX Center Frequency: %f MHz...") % (freq/1e6) << std::endl;
  uhd::tune_request_t tune_request(freq);
  if (vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
  usrp->set_rx_freq(tune_request);
  std::cout << boost::format("Actual RX Center Freq Acquired: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl;

  // Set the RX Gain of the receivers front end
  std::cout << boost::format("Requesting RX Gain: %f dB...") % gain << std::endl;
  usrp->set_rx_gain(gain);
  std::cout << boost::format("Actual RX Gain Acquired: %f dB...") % usrp->get_rx_gain() << std::endl;

  if (vm.count("ant")) usrp->set_rx_antenna(ant);

  // sleep for 2 seconds while hardware catches up..
  boost::this_thread::sleep(boost::posix_time::seconds(2)); //allow for some setup time

  // check that the LO on the radio are locked and working correctly
  std::vector<std::string> sensor_names;
  sensor_names = usrp->get_rx_sensor_names(0);
  if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end() ) {
    uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked", 0);
    std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string() << std::endl;
    UHD_ASSERT_THROW(lo_locked.to_bool()); /* throw excpetion if LO is not locked.. */
  }

  // create receive streamer
  uhd::stream_args_t stream_args("fc32"); // complex floats
  uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

  // setup streaming
  uhd::stream_cmd_t stream_cmd( uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS );
  stream_cmd.num_samps = 0; /*inf*/
  stream_cmd.stream_now = true;
  rx_stream->issue_stream_cmd(stream_cmd);

  uhd::rx_metadata_t md;
  std::vector<std::complex<float> > buff(rx_stream->get_max_num_samps());
  
  bool isRunning = true;
  int timeout_counter = 0;
  std::vector<double> integration_windows;

  while( isRunning ) {
    size_t num_rx_samps = rx_stream->recv( &buff.front(), buff.size(), md );

    // handle errors while streaming
    switch( md.error_code ) {
      case uhd::rx_metadata_t::ERROR_CODE_NONE:
        timeout_counter = 0;
        break;
      case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
        std::cout << "USRP Read Timeout!" << std::endl;
        timeout_counter++;
        if ( timeout_counter > 5 ) {
          std::cout << "Timeouts allow exceeded, shutting down.." << std::endl;
          isRunning = false;
        }
        break;
      case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
        std::cout << "Overflow.." << std::endl;
        break;
      default:
        // unknown or other error occured..
        std::cout << boost::format( "Got unknown error code 0x%x, exiting receive loop...") % md.error_code << std::endl;
        isRunning = false;
    }
    /////////
    // DO something with the samples
    ////////
    integration_windows.push_back( measure_rx_power( buff ) );
    if ( integration_windows.size() == windows_count ) {
      // got Y windows, average and compute power
      double power_dBm = windows_to_power_dBm( integration_windows );
      std::cout << "Measured Power: " << power_dBm << " dBm. " << std::endl;
      integration_windows.clear();
    }
  }
  return 0;
}

double windows_to_power_dBm( std::vector<double> &pm )
{
  // windows contain rms power values
  double pwr_rms = 0.0;
  for ( auto it = pm.begin(); it != pm.end(); ++it)
  {
    pwr_rms += *it;
  }
  pwr_rms = pwr_rms / pm.size();
  return 10*log(pwr_rms)+30;
}

double measure_rx_power( std::vector<std::complex<float> > &buff )
{
  // rms power in digital samples
  // is the sqrt of the mean of the magnatude
  double mag_avg;
  for (auto it = buff.begin(); it != buff.end(); ++it)
  {
    mag_avg += std::abs(*it);
  }
  mag_avg = mag_avg / buff.size();
  double power_rms = sqrt(mag_avg*mag_avg)/100;  /* 50 or 100 is typical impedances */
  return power_rms;
}

