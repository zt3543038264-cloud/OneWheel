import importlib.util, socket, struct, threading, unittest, zlib
from pathlib import Path
spec=importlib.util.spec_from_file_location('backup',Path(__file__).with_name('read_flash.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class BackupTests(unittest.TestCase):
 def test_crc(self):
  w=[0]*1024;w[:4]=[0x51435453,1,0xB50A9F5F,0x3F999991];w[4]=zlib.crc32(struct.pack('<4I',*w[:4]));self.assertTrue(m.validate(w)['crc_matches']);self.assertTrue(m.validate(w)['zero_tail']);w[3]^=1;self.assertFalse(m.validate(w)['crc_matches'])
 def test_udp_double_read(self):
  with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as server,socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as client:
   server.bind(('127.0.0.1',0));server.settimeout(3);client.bind(('127.0.0.1',0));client.settimeout(.2);errors=[]
   def serve():
    try:
     for _ in range(256):
      raw,addr=server.recvfrom(200);op,seq,at=raw.decode().strip().split(',');assert op=='cfg:flash_read';at=int(at);assert at%8==0 and 0<=at<=1016
      server.sendto((f'rsp:{seq},ok,flash_read,{at},'+','.join(f'{i:08X}' for i in range(at,at+8))+'\n').encode(),addr)
    except Exception as e:errors.append(e)
   t=threading.Thread(target=serve);t.start()
   self.assertEqual(m.download(client,server.getsockname(),1000),list(range(1024)))
   self.assertEqual(m.download(client,server.getsockname(),2000),list(range(1024)));t.join();self.assertFalse(errors)
 def test_error_reply(self):
  class Fake:
   def sendto(self,*a):pass
   def recvfrom(self,*a):return b'rsp:1,err,BUSY,Stop required\n',('127.0.0.1',9)
  with self.assertRaises(RuntimeError):m.request(Fake(),('127.0.0.1',9),1,0)
if __name__=='__main__':unittest.main()
