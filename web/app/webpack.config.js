const path = require('path');

module.exports = (env) => {
  return {
    entry: './index.js',
    output: {
      filename: 'main.js',
      path: path.resolve(__dirname, '../public/js')
    }
  }
};
