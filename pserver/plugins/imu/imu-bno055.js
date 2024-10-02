module.exports = {
	create_plugin: function (plugin_host) {
		console.log("create imu_bno055 plugin");
    const i2c = require('i2c-bus');
    let m_options = {};

		var plugin = {
			name: "imu_bno055",
			init_options: function (options) {
				m_options = options["imu_bno055"];

				if (m_options && m_options.enabled) {
					plugin.start_read();
				}
			},
      start_read: () => {

        // BNO055のI2Cアドレス
        const BNO055_I2C_ADDR = 0x29; // (プライマリアドレス: 0x28, セカンダリ: 0x29)
        
        // BNO055のレジスタ
        const BNO055_CHIP_ID = 0x00;
        const BNO055_OPR_MODE = 0x3D;
        const BNO055_PWR_MODE = 0x3E;
        const BNO055_SYS_TRIGGER = 0x3F;
        const BNO055_UNIT_SEL = 0x3B;
        const BNO055_EULER_H_LSB = 0x1A;
        const BNO055_QUATERNION_DATA_W_LSB = 0x20; // クォータニオンデータの開始レジスタ
        
        // BNO055の操作モード (NDOF: 9DOFフュージョン)
        const OPERATION_MODE_NDOF = 0x0C;
        
        // 初期化関数
        function initBNO055(bus_num) {
          
          let i2cBus;
          try {

            i2cBus = i2c.openSync(bus_num);

            // チップIDを確認
            const chipId = i2cBus.readByteSync(BNO055_I2C_ADDR, BNO055_CHIP_ID);
            if (chipId !== 0xA0) {
              throw new Error('BNO055のチップIDが一致しません');
            }
          
            // センサーの初期化
            i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_PWR_MODE, 0x00); // 通常動作モード
            i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_SYS_TRIGGER, 0x00); // リセット解除
            i2cBus.writeByteSync(BNO055_I2C_ADDR, BNO055_OPR_MODE, OPERATION_MODE_NDOF); // NDOFモード設定
            console.log('BNO055 初期化完了');
      
          } catch (error) {
            console.error("Error:", error);
          } finally {
            if (i2cBus) {
              try {
                i2cBus.closeSync();
              } catch (closeError) {
                console.error("Failed to close I2C bus:", closeError);
              }
            }
          }
        }
        
        // データ取得関数 (クォータニオン)
        function readQuaternion(bus_num) {
          
          let i2cBus;
          try {

            i2cBus = i2c.openSync(bus_num);

            const buffer = Buffer.alloc(8); // 8バイト分のバッファ
            i2cBus.readI2cBlockSync(BNO055_I2C_ADDR, BNO055_QUATERNION_DATA_W_LSB, 8, buffer);
          
            // 生の16ビット整数値を取得
            let w = (buffer[1] << 8) | buffer[0];
            let x = (buffer[3] << 8) | buffer[2];
            let y = (buffer[5] << 8) | buffer[4];
            let z = (buffer[7] << 8) | buffer[6];
          
            // 2の補数形式で扱うため、負の値が正しく処理されるようにする
            if (w & 0x8000) w -= 65536;
            if (x & 0x8000) x -= 65536;
            if (y & 0x8000) y -= 65536;
            if (z & 0x8000) z -= 65536;
          
            // クォータニオンの正規化 (1単位 = 1/2^14)
            const scale = 1.0 / (1 << 14); // 1/16384
            return {
              w: w * scale,
              x: x * scale,
              y: y * scale,
              z: z * scale,
            };
      
          } catch (error) {
            console.error("Error:", error);
          } finally {
            if (i2cBus) {
              try {
                i2cBus.closeSync();
              } catch (closeError) {
                console.error("Failed to close I2C bus:", closeError);
              }
            }
          }
        }
        
        // クォータニオンから方位（ヘディング、Yaw）を計算する関数
        function quaternionToHeading(quaternion) {
          const w = quaternion.w;
          const x = quaternion.x;
          const y = quaternion.y;
          const z = quaternion.z;
        
          // ヘディング（Z軸周りの回転）を計算
          const t3 = +2.0 * (w * z + x * y);
          const t4 = +1.0 - 2.0 * (y * y + z * z);
          let heading = Math.atan2(t3, t4) * (180 / Math.PI); // ラジアンを度に変換
        
          // ヘディングを逆にする（符号を反転）
          heading = -heading;
        
          // 0〜360度の範囲に調整
          if (heading < 0) {
            return heading + 360;
          }
          return heading;
        }
        
        // メイン処理
        initBNO055(0);
        setInterval(() => {
          try {
            // クォータニオンを取得
            const quaternion = readQuaternion(0);
            if(m_options.debug){
              console.log('クォータニオン');
              console.log('W:', quaternion.w);
              console.log('X:', quaternion.x);
              console.log('Y:', quaternion.y);
              console.log('Z:', quaternion.z);
            }
        
            // クォータニオンから方位を計算
            const heading = quaternionToHeading(quaternion);
            if(m_options.debug){
              console.log('方位 (Heading):', heading);
            }

            const imu_value = {
              heading,
              quaternion,
            };

            if (m_plugin_host.get_redis_client) {
              const client = m_plugin_host.get_redis_client();
              if (client) {
                client.publish(`pserver-imu`, JSON.stringify(imu_value), (err, reply) => {
                    if (err) {
                        console.error('Error publishing message:', err);
                    } else {
                        //console.log(`Message published to ${reply} subscribers.`);
                    }
                });
              }
            }
          } catch (error) {
            console.error('エラー:', error.message);
          }
        }, 1000); // 1秒ごとに取得
      }
		};
		return plugin;
	}
};
