var fs = require("fs");
const yaml = require("js-yaml");
let SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");
let comport;
let port;

let parser;

function resolveAfter2Seconds(str) {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve(str);
      console.log(str); // 10
    }, 500);
  });
}

async function f1(str) {
  var x = await resolveAfter2Seconds(str);
}

const callerFun = async () => {
  async function write(str, result) {
    return new Promise((resolve) => {
      port.write(`${str}`, function () {
        resolve(result);
      });
    });
  }

  async function sleep2(delay, result) {
    return new Promise((resolve) => {
      setTimeout(() => resolve(result), delay);
    });
  }

  while (true) {
    const files = fs.readdirSync(__dirname + "/maze_data");
    var list = [];
    for (var i in files) {
      if (files[i].match(/.yaml$/)) {
        list.push(files[i]);
      }
    }
    for (var i in list) {
      console.log(` ${i} : ${list[i]}`);
    }
    console.log(">");
    var str = fs.readFileSync("/dev/stdin").toString().trim();

    var idx = parseInt(str);
    if (str === "all") {
      console.log("all")
      for (const file of files) {
        if (file.match(/.yaml$/)) {
          let txt = fs.readFileSync(`${__dirname}/maze_data/${file}`, {
            encoding: "utf-8",
          });
          var file_name = file.replace("yaml", "txt");
          var saveData = yaml.load(txt);
          var str = `${file_name}@${JSON.stringify(saveData)}`;
          write(str);
          // console.log(saveData)
          await sleep2(600);
          console.log(`${file}: finish!!`);
        }
      }
    } else {
      if (idx > list.length) {
        console.log("out of index");
        continue;
      }
      let txt = fs.readFileSync(`${__dirname}/maze_data/${list[idx]}`, {
        encoding: "utf-8",
      });
      var file_name = list[idx].replace("yaml", "txt");
      var saveData = yaml.load(txt);
      var wall = new Array(saveData.maze_data.maze_size);
      for (var i = 0; i < saveData.maze_data.maze_size; i++) {
        wall[i] = new Array(saveData.maze_data.maze_size)
      }
      var c = 0;
      for (var i = 0; i < saveData.maze_data.maze_size; i++) {
        for (var j = 0; j < saveData.maze_data.maze_size; j++) {
          wall[i][j] = saveData.maze_data.wall[c];
          c++;
        }
      }
      var maze_data = "";
      for (var i = 0; i < saveData.maze_data.maze_size; i++) {
        for (var j = 0; j < saveData.maze_data.maze_size; j++) {
          maze_data += `${(wall[j][i] | 0xf0)},`;
        }
      }
      console.log(maze_data)
      var str = `maze.log@${maze_data} `;
      write(str);
      // console.log(saveData)
      await sleep2(600);
      console.log(`${list[idx]}: finish!!`);
    }
  }
};

let ready = function () {
  port = new SerialPort(
    comport,
    {
      baudRate: 3000000,
      // baudRate: 115200,
    },
    (e) => {
      if (e) {
        console.log("comport access dinied");
      } else {
        console.log("connect");
        callerFun();
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
    console.log(data);

    if (data.match(/^end___/)) {
      obj.dump_to_csv = false;

      console.log(`${__dirname} /logs/${obj.file_name} `);

      fs.writeFileSync(`${__dirname} /logs/${obj.file_name} `, `${obj.record} `, {
        flag: "w+",
      });

      fs.copyFileSync(
        `${__dirname} /logs/${obj.file_name} `,
        `${__dirname} /logs/latest.csv`
      );
    }
    if (obj.dump_to_csv) {
      obj.record += `${data} \n`;
    }

    if (data.match(/^start___/)) {
      obj.dump_to_csv = true;
      obj.file_name = getNowYMD();
      obj.record = "";
      console.log(obj);
    }
  });
};

SerialPort.list().then(
  (ports) => {
    for (let i in ports) {
      const p = ports[i];
      console.log(p.path, p.serialNumber);
      if (
        p.path.match(/usbserial/) ||
        p.path.match(/COM/) ||
        p.path.match(/ttyUSB/)
      ) {
        if (p.serialNumber) {
          comport = p.path;
          console.log(`select: ${comport} `);
          ready();
          break;
        }
      }
    }
  },
  (err) => console.error(err)
);
