#ifndef ENUMS_HPP
#define ENUMS_HPP

enum class SearchMode : int {
  ALL = 0,
  Kata = 1,
  Return = 2,
};

enum class LogFileType : int {
  SLALOM = 0,
  STRAIGHT = 1,
};

enum class SensorCtrlType : int {
  NONE = 0,
  Straight = 1,
  Dia = 2,
};

enum class RUN_MODE2 : int {
  NONE_MODE = 0,
  KEEP = 0,
  SLAROM_RUN = 1,
  PIVOT_TURN = 2,
  ST_RUN = 3,
  SLALOM_RUN2 = 4,
  FRONT_CTRL = 5,
};

enum class MotionType : int {
  NONE = 0,
  STRAIGHT = 1,
  PIVOT = 2,
  SLA_FRONT_STR = 3,
  SLALOM = 4,
  BACK_STRAIGHT = 5,
  WALL_OFF = 6,
  READY = 7,
  PIVOT_PRE = 8,
  PIVOT_PRE2 = 9,
  PIVOT_AFTER = 10,
  FRONT_CTRL = 11,
  PIVOT_OFFSET = 12,
  WALL_OFF_DIA = 13,
  SLA_BACK_STR = 14,
  SYS_ID_PARA = 15,
  SYS_ID_ROLL = 16,
  SENSING_DUMP = 17,
};

enum class FailSafe : int {
  NONE = 0,
  ERROR = 1,
};
enum class MotionResult : int {
  NONE = 0,
  ERROR = 1,
};
enum class SearchResult : int {
  SUCCESS = 0,
  FAIL = 1,
};

enum class MotionDirection : int {
  NONE = 0,
  RIGHT = 1,
  LEFT = 2,
};

enum class WallOffReq : int {
  NONE = 0,
  SEARCH = 1,
  FAST_RUN = 2,
};

enum class WallCtrlMode : int {
  NONE = 0,
  LEFT_ONLY = 1,
  RIGHT_ONLY = 2,
  BOTH = 3,
};

#endif