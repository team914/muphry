"use strict";

Object.defineProperty(exports, "__esModule", {
  value: true
});
exports.default = void 0;

var _execa = _interopRequireDefault(require("execa"));

var _fs = _interopRequireDefault(require("fs"));

var _hook = _interopRequireDefault(require("../commands/hook/hook"));

function _interopRequireDefault(obj) { return obj && obj.__esModule ? obj : { default: obj }; }

const isHookCreated = async () => {
  try {
    const {
      stdout
    } = await (0, _execa.default)('git', ['rev-parse', '--absolute-git-dir']);

    if (!_fs.default.existsSync(stdout + _hook.default.PATH)) {
      return false;
    }

    return _fs.default.readFileSync(stdout + _hook.default.PATH) === _hook.default.CONTENTS;
  } catch (error) {
    console.error(error);
  }
};

var _default = isHookCreated;
exports.default = _default;