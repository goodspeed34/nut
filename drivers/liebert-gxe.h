#ifndef _GXE_UPS_H_
#define _GXE_UPS_H_

enum gxe_cmds {
	GXE_FIRST = 0,
	GXE_GET_ANALOG_DATA,
	GXE_GET_ONOFF_DATA,
	GXE_GET_WARNING_DATA,
	GXE_REMOTE_COMMAND,
	GXE_GET_SYS_PARAM,
	GXE_SET_SYS_PARAM,
	GXE_GET_PROTO_VER,
	GXE_GET_DEV_ADDR,
	GXE_GET_VENDOR_INFO,
	GXE_GET_VENDOR_VER,
	GXE_GET_FW_VER,
	GXE_PARA_ANALOG_DATA,
	GXE_LAST,
};

char gxe_cmds[GXE_LAST][4] = {
	{'0','0','0','0'},
	{'2','A','4','2'},
	{'2','A','4','3'},
	{'2','A','4','4'},
	{'2','A','4','5'},
	{'2','A','4','7'},
	{'2','A','4','9'},
	{'2','A','4','F'},
	{'2','A','5','0'},
	{'2','A','5','1'},
	{'2','A','8','0'},
	{'2','A','E','5'},
	{'2','A','E','6'},
};

char *rtn_vals[] = {
	"OK",
	"Bad VER",
	"Bad CHKSUM",
	"Bad LCHKSUM",
	"Invalid CID2",
	"Bad Command Format",
	"Bad Data",
};

#define RTN_TO_STR(rtn)							\
  (((size_t) rtn) < sizeof(rtn_vals)/sizeof(*rtn_vals) ? rtn_vals[(rtn)] : "Unknown RTN")

#endif /* _GXE_UPS_H_ */
