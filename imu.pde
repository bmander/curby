int lf=10;

class IMUParseException extends Exception{
  IMUParseException(String msg){
    super(msg);
  }
}

class IMUReading {
  int ax;
  int ay;
  int az;
  int wx;
  int wy;
  int wz;
  int t;
  
  IMUReading(String line) throws IMUParseException{
    String[] fields = split(trim(line),",");
    if(fields.length!=7){
      throw new IMUParseException("malformed IMU reading");
    }
    ax=int(fields[0]);
    ay=int(fields[1]);
    az=int(fields[2]);
    wx=int(fields[3]);
    wy=int(fields[4]);
    wz=int(fields[5]);
    t=int(fields[6]);
  }
  
  String toString(){
    return "["+ax+","+ay+","+az+","+wx+","+wy+","+wz+","+t+"]";
  }
}

class IMU{
  Serial serial;
  
  IMU(PApplet sketch, String port){
    serial = new Serial(sketch, serialPort, 115200);
    serial.clear();
  }
  
  String readRaw(){
    String chunk = serial.readStringUntil(lf);
    return chunk;
  }
  
  void clear(){
    serial.clear();
  }
  
  IMUReading read() throws IMUParseException{
    String raw = readRaw();
    if(raw==null){
      return null;
    }
    return new IMUReading( raw );
  }
}
