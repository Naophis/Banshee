var fs = require("fs");
const yaml = require("js-yaml");
let SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");

let comport;
let port;

let parser;

let rorl = process.argv[2].toUpperCase();
let dist = parseFloat(process.argv[3]);
console.log(`dist,L90,L45,F,R45,R90`);
let ready = function () {
  port = new SerialPort(
    comport,
    {
      baudRate: 2000000,
      // baudRate: 115200,
    },
    (e) => {
      if (e) {
        console.log("comport access dinied");
      } else {
        // console.log("connect");
      }
    }
  );
  parser = port.pipe(
    new Readline({
      delimiter: "\r\n",
    })
  );
  function getNowYMD() {
    var dt = new Date();
    var y = dt.getFullYear();
    var m = ("00" + (dt.getMonth() + 1)).slice(-2);
    var d = ("00" + dt.getDate()).slice(-2);
    var h = ("00" + dt.getHours()).slice(-2);
    var M = ("00" + dt.getMinutes()).slice(-2);
    var s = ("00" + dt.getSeconds()).slice(-2);
    return `${y}${m}${d}_${h}${M}_${s}.csv`;
  }
  let obj = {
    dump_to_csv: false,
    file_name: getNowYMD(),
    record: "",
  };

  parser.on("data", function (data) {
    console.log(`${dist},${data}`);
  });
};

SerialPort.list().then(
  (ports) => {
    for (let i in ports) {
      const p = ports[i];
      // console.log(p.path, p.serialNumber);
      if (
        p.path.match(/usbserial/) ||
        p.path.match(/COM/) ||
        p.path.match(/ttyUSB/)||
        p.path.match(/ttyACM/)
      ) {
        if (p.serialNumber) {
          comport = p.path;
          // console.log(`select: ${comport}`);
          ready();
          break;
        }
      }
    }
  },
  (err) => console.error(err)
);
