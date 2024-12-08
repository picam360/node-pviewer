
class EncoderOdometry {
    constructor() {
        this.x = 0;
        this.y = 0;
    }
  
    push(pif_buffer) {
    }

    getCurrentPosition(){
        return {
            x : this.x,
            y : this.y
        };
    }
  }
  
  module.exports = EncoderOdometry;