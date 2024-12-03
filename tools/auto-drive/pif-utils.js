const fs = require("fs");
const path = require('path');
const fxp = require('fast-xml-parser');

function read_pif(file_path, callback) {
	try{
		const data = fs.readFileSync(file_path);
		// 最初の2バイトを取得
		const header_head = data.slice(0, 2).toString('utf-8');
		if (header_head !== 'PI') {
			throw new Error('Invalid file format');
		}

		// 次の4バイトでサイズを取得（ビッグエンディアン）
		const header_size = data.readUInt16BE(2);
		const header = data.slice(0, header_size + 4);

		// XMLデータを取得してパース
		const xmlData = header.slice(4).toString('utf-8');
        const parser = new fxp.XMLParser({
            ignoreAttributes: false,
            attributeNamePrefix: "",
        });
        const img_dom = parser.parse(xmlData);

        const meta_size = parseInt(img_dom["picam360:image"].meta_size, 10);
        const meta = data.slice(4 + header_size, 4 + header_size + meta_size);
        const jpeg_filepath = file_path + ".0.0.JPEG";
        const jpeg_data = fs.readFileSync(jpeg_filepath);
        callback(file_path, [header, meta, jpeg_data]);
	}catch(err){

	};
}


module.exports = {
    read_pif,
};
