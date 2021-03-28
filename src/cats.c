/**
 * @file cats.c
 * @brief Main daemon code
 *
 * @author Michel Blanc
 * @date 2009-11-14
 *
 * @version 1.0
 *
 * - Project: cats
 * - Tabsize: 2
 * - Copyright: (c) 2009 Michel Blanc <mb@mbnet.fr>
 * - License: GNU GPL v2
 * - Part of this code (c) ERASME
 *
 * This program is a very basic cat-like utility for serial port.
 * Works on Rx and Tx.
 *
 * This code is distributed under the GNU Public License
 * which can be found at http://www.gnu.org/licenses/gpl.txt
 *
 * @sa cats.c
 */

#include <ctype.h>
#include <fcntl.h> 
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/time.h> 
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#define VERSION 1.0
#define BAUD_DEFAULT B9600

/** 
 * @brief Boolean type 
 *
 */
#ifdef TRUE
#  ifndef boolean_t
#    define boolean_t int
#  endif
#else
#  ifdef boolean_t
#    define FALSE 0
#    define TRUE 1
#  else
     typedef enum {FALSE, TRUE} boolean_t;
#  endif
#endif

int            oDebug = 2;
boolean_t      oLineTimings = FALSE;
struct timeval oStartTime;

/**
 * @brief Prints messages to console according to severity.
 *
 * Print the messaged passed in arguments if it's level is 
 * higher of equal to the threshold level specified at command line.
 * The level values can usualy be found in /usr/include/sys/syslog.h
 * @param level
 *  The level for this message
 * @param template
 *  A variable argument list containing printf style statements.
 * @sa /usr/include/sys/syslog.h
 */
static void 
debug(int level, const char * template, ...)
{
  va_list ap;

  va_start (ap, template);

  if (oDebug < level) return;
  vfprintf(stderr, template, ap);
  fprintf(stderr,"\n");
  fflush(stderr);

  va_end (ap);

  return;
}

/**
 * @brief Initialize the serial port with some tweaks.
 *
 * Open the serial port and set parameters for it.
 * Mainly, it is set at 8 data bits, no parity, 1 stop bit.
 * Port is also set RAW, and VMIN is set to 2 since all data
 * that need to be read has even bytes.
 *
 * @param fd 
 *  The file descriptor to read from
 * @param baudrate 
 *  The baudrate to set
 * @return 
 *  TRUE if happy, FALSE if something went wrong.
 * @warning 
 *  This function should not be used externally
 */
boolean_t 
serialInitPort(int fd, int baudrate) 
{

  // use :   fd = open(port, O_RDWR | O_NOCTTY);

  struct termios options;

  debug(LOG_DEBUG,"Setting port attributes");

  /* Get the current options for the port */
  tcgetattr(fd, &options);

  /* Set the baud rates to BAUDRATE */
  if (cfsetispeed(&options, baudrate) == -1) {
    debug(LOG_WARNING,"cfsetispeed error for baudrate %d", baudrate);
    return FALSE;
  }

  if (cfsetospeed(&options, baudrate)) {
    debug(LOG_WARNING,"cfsetospeed error for baudrate %d", baudrate);
    return FALSE;
  }

  /* 8N1 */
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  /* No hardware flow control */
  options.c_cflag &= ~CRTSCTS;
  
  /* Turn on READ & ignore ctrl lines */
  options.c_cflag |= CREAD | CLOCAL;  

  /* No software flow control */
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  
  /* Make raw */
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;
  
  /* see: http://unixwiz.net/techtips/termios-vmin-vtime.html */
  options.c_cc[VMIN]   = 1; 
  options.c_cc[VTIME]  = 0; 
  
  /* Apply the new options for the port */
  tcsetattr(fd, TCSANOW, &options);

  return TRUE;
}

/**
 * @brief Serial device lookup and opening
 *
 * Opens the device given in parameters. If device is NULL, 
 * this function will try to autodetect the port (NOT IMPLEMENTED).
 *
 * @param[out] fd The file descriptor to read to/write from is open is successful.
 * @param[in] port The serial rort device to use (e.g. "/dev/ttyUSB1")
 * @param[in] baudrate The communication baud rate (only 9600, 19200 and 57600 are supported)
 *
 * @return 0 on failure, a real fd number otherwise.
 *
 */

int 
serialOpen(const char *port, int baudrate) {
  int baud_sym;

  int fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  //  fcntl(fd, F_SETFL, FNDELAY);

  debug(LOG_DEBUG,"Opening port %s...", port);

  if (fd == -1) {
    debug(LOG_ALERT,"error: port %s opening failed. Permission problem ?", port);
    exit(EXIT_FAILURE);
  } 

  fcntl(fd, F_SETFL, 0);
  debug(LOG_INFO,"Port %s opened successfuly", port);

  /* Set port attributes (baudrate only) */
  // Check that argv[argc-1] 
  switch (baudrate) {
  case 50: baud_sym = B50; break;
  case 75: baud_sym = B75; break;
  case 110: baud_sym = B110; break;
  case 134: baud_sym = B134; break;
  case 150: baud_sym = B150; break;
  case 200: baud_sym = B200; break;
  case 300: baud_sym = B300; break;
  case 600: baud_sym = B600; break;
  case 1200: baud_sym = B1200; break;
  case 1800: baud_sym = B1800; break;
  case 2400: baud_sym = B2400; break;
  case 4800: baud_sym = B4800; break;
  case 9600: baud_sym = B9600; break;
  case 19200: baud_sym = B19200; break;
  case 38400: baud_sym = B38400; break;
  case 57600: baud_sym = B57600; break;
  case 115200: baud_sym = B115200; break;
  case 230400: baud_sym = B230400; break;
  case 460800: baud_sym =  B460800; break;
  case 500000: baud_sym =  B500000; break;
  case 576000: baud_sym =  B576000; break;
  case 921600: baud_sym =  B921600; break;
  case 1000000: baud_sym =  B1000000; break;
  case 1152000: baud_sym =  B1152000; break;
  case 1500000: baud_sym =  B1500000; break;
  case 2000000: baud_sym =  B2000000; break;
  case 2500000: baud_sym =  B2500000; break;
  case 3000000: baud_sym =  B3000000; break;
  case 3500000: baud_sym =  B3500000; break;
  case 4000000: baud_sym =  B4000000; break;
  default: printf("Error : unsupported baud rate"); exit(EXIT_FAILURE);
  }

  serialInitPort(fd, baud_sym);

  debug(LOG_INFO,"Baudrate set successfuly to %d", baudrate);

  return fd;
}

/**
 * @brief Usage of command line switches
 *
 * Usually called when a wrong command line switch is used, or
 * when a mandatory option is missing (but there is not as of now).
 */

void 
usage(void)
{
  fprintf(stdout, "cats, version %.2f\n\n",VERSION);
  fprintf(stdout, "  Usage : cats [-h] [-g <0-7>] [-b <baud>] [-t] <port>\n\n");

  fprintf(stderr, "\t-h : this help\n");
  fprintf(stderr, "\t<port> : serial port (default : none)\n");
  fprintf(stderr, "\t-b <rate> : serial baud rate (default : 9600)\n");
  fprintf(stderr, "\t-t : write timestamp in front on each line\n");
  fprintf(stderr, "\t-g <0-7> : debug level (default : no debug, see sys/syslog.h for levels)\n");
  fprintf(stderr, "\n  Supported baud rates :\n");
  fprintf(stderr, "\t\t50 75 110 134 150 200 300 600 1200 1800\n");
  fprintf(stderr, "\t\t2400 4800 9600 19200 38400 57600 115200 230400\n");
  fprintf(stderr, "\t\t230400 460800 500000 576000 921600 1000000 1152000 1500000\n");
  fprintf(stderr, "\t\t2000000 2500000 3000000 3500000 4000000\n");
  fprintf(stderr, "\n  Example : cats -g7 -b19200 /dev/ttyUSB0\n");

  fprintf(stderr,"\n");

  exit(EXIT_SUCCESS);
}

/**
 * @brief Compute elapsed
 *
 * computes time elapsed between now and and a timevazl argument passed
 *
 * @param starttime
 *  The time to compare to now (should be in the past)
 * @param result
 *  A timeval struct where the result is put
 *
 */
int
elapsedFromStart (struct timeval *starttime, struct timeval *result) {

  struct timeval now;
  gettimeofday(&now, NULL);

  result->tv_sec = now.tv_sec - starttime->tv_sec;
  result->tv_usec = now.tv_usec - starttime->tv_usec;

  if (result->tv_usec < 0) {
    result->tv_sec -= 1;
    result->tv_usec += 1000000;
  }

  /* Return 1 if result is negative. */
  return now.tv_sec < starttime->tv_sec;
}

/**
 * @brief Never ending handling loop
 *
 * This never ending loop copies STDIN to Tx and Rx to STDOUT
 * until the end of times
 *
 * @param serialPort 
 *  The serial port device to read and write data from/to
 * @param baudRate
 *  The serial baudrate to use
 *
 */
void 
serialLoop(const char *serialPort, int baudRate) {
  int            fd;
  char           data[1024];
  char           splitbuf[1024];
  int            n;
  int            nbytes;
  int            max_fd;
  fd_set         input;
  struct timeval timeout;
  boolean_t      linebreak = FALSE;
  int            i;
  int            bcount;
  struct timeval elapsed;
  struct timeval starttime;

  fd = serialOpen(serialPort, baudRate);
  
  /* Initialize the input set */
  FD_ZERO(&input);

  gettimeofday(&starttime, NULL);

  while (1) {
    FD_SET(fileno(stdin), &input); // stdin
    FD_SET(fd, &input);
    
    max_fd = fd + 1;
    
    /* Initialize the timeout structure */
    timeout.tv_sec  = 10;
    timeout.tv_usec = 0;
    
    /* Do the select */
    n = select(max_fd, &input, NULL, NULL, &timeout);

    /* See if there was an error */
    if (n < 0) {
      debug(LOG_DEBUG, "select critical");
      perror("select failed");
    } else if (n == 0) {
      debug(LOG_DEBUG, "select timeout");
    } else {
      /* We have input */
      if (FD_ISSET(fd, &input)) {

        bzero(data,1024);
        bzero(splitbuf,1024);

        if ((bcount = read(fd, data, 1024))) {
          for (i=0; i < bcount; i++) {
            if (linebreak && oLineTimings) {
              elapsedFromStart(&starttime, &elapsed);
              printf("[%ld.%06ld]", elapsed.tv_sec, elapsed.tv_usec);
            }
            linebreak = (data[i] == '\n' ? TRUE : FALSE);
            printf("%c", data[i]);
            fflush(stdout);
          }
        }
      }
      if (FD_ISSET(0, &input)) {
        bzero(data,1024);
        nbytes = read(fileno(stdin), data, 1024);
        write(fd, data, nbytes);
      }
    }
  }
}

/**
 * @brief Well, this is main...
 *
 * Just handling cmd line arguments and starting the serial loop.
 *
 * @return Hopefuly not
 */
int 
main(int argc, char **argv)
{
  char c;
  int  oSerialBaudRate = 0;

  if (argc == 1)
    usage();

  /* thread stuff */
  while ((c = getopt(argc, argv, "hg::b:t")) != -1 ) {
    debug(LOG_DEBUG, "handling option %c",c);
    switch (c) {
    case 'g':
      oDebug = atoi(optarg);
      break;
    case 'b':
      oSerialBaudRate = atoi(optarg);
      break;
    case 't':
      oLineTimings = TRUE;
      break;
    case '?':
      if (isprint (optopt))
          fprintf (stderr, "Unknown option `-%c'.\n", optopt);
      else
        fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
    case 'h':
    default:
      usage();
    }
  }

  if (!oSerialBaudRate) {
    debug(LOG_NOTICE, "baud rate unspecified, using 9600");
    oSerialBaudRate = 9600;
  }

  debug(LOG_DEBUG, "finished handling options");

  if (!strstr(argv[argc-1],"/dev/tty")) {
    debug(LOG_WARNING, "invalid serial port specified : %s", argv[argc-1]);
  }
  
  serialLoop(argv[argc-1], oSerialBaudRate);

  return 0;
}

/** @mainpage cats Documentation
 *
 * @section intro_sec Introduction
 *
 * If you fiddle sometimes with digital electronics and serial enabled gadgets,
 * you might lack a simple and straightforward command line utility to "see"
 * what happens on the serial port. Wait no more, cats is here.
 *
 * @section install_sec Compiling
 *
Compiling cats should be as simple as typing 'make'.
There is not requirements for fancy libraries or exotic 
dependencies.
 *  
 * @section running Running
 *
 * Cats supports few command line switches and arguments.
 *
 * -h : asks for help
 * <port> : which serial port tu use (default : none)
 * -b <rate> : which serial baud rate to use (default : 9600)
 * -g <0-7> : debug level (default : no debug, see sys/syslog.h for levels)
 *
 * The list of supported baud rates is rather extensive, but mignt not cover all cases. For instance, playing with MIDI (31250) is not supported for now, but since MIDI is a binary protocol, it's not a problem right now.
 *  50 75 110 134 150 200 300 600 1200 1800 2400 4800 9600 19200 38400 57600 115200 230400
 *  230400 460800 500000 576000 921600 1000000 1152000 1500000 2000000 2500000 3000000 3500000 4000000
 *
 * @section usage Usage Exemples
 *
 * Dump what goes in our /dev/ttyUSB0 serial port at 9600 bps : cats /dev/ttyUSB0
 *
 *
 * Dump what goes in our /dev/ttyUSB1 serial port at 19200 bps : cats -b19200 /dev/ttyUSB1
 *
 * Writing to the serial port : echo "hello" | cats /dev/ttyS2
 *
 * Poor man's serial R/W console (yes, you can configure your Cisco box this way) : cat | cats /dev/ttyS1
 *
 * etc...
 * 
 * @section usage Calling for Halp!
 *
 * Questions, requests, flames, etc... can be directed at mb@mbnet.fr, or preferably
 * at the GitHub cats's home.
 *
 */


/* Local Variables: */
/* mode:c           */
/* comment-column:0 */
/* compile-command: "gcc cats.c -o cats" */
/* tab-width: 2 */
/* End:             */
