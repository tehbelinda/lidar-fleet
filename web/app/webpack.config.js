const path = require('path');
const MiniCssExtractPlugin = require('mini-css-extract-plugin');

module.exports = (env) => {
  return {
    entry: './src/index.jsx',
    output: {
      filename: 'bundle.js',
      path: path.resolve(__dirname, '../public/js')
    },
    devtool: 'inline-source-map',
    devServer: {
      contentBase: path.join(__dirname, '../public'),
      publicPath: path.resolve(__dirname, '/js/')
    },
    module: {
        rules: [
          {
            test: /\.(js|jsx)$/,
            exclude: /node_modules/,
            use: ['babel-loader']
          },
          {
            test: /\.scss$/,
            use: [
              // fallback to style-loader in development
              process.env.NODE_ENV !== 'production' ? 'style-loader' : MiniCssExtractPlugin.loader,
              "css-loader",
              "sass-loader"
            ]
          }
        ]
      },
      plugins: [
        new MiniCssExtractPlugin({
          filename: 'bundle.css'
        })
      ],
      resolve: {
        extensions: ['.js', '.jsx', '.css']
      },
  }
};
