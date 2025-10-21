# encoding: utf-8
'''
Company: 深圳市幻尔科技有限公司(Shenzhen Hiwonder Technology Co., Ltd.)
官网:hiwonder.com(official website:hiwonder.com)
日期:2019/9/23(date:2019/9/23)
by Aiden
'''
'''
最大支持32G的sd卡(support SD cards up to 32G)
支持FAT16, FAT32文件系统(support FAT16 and FAT32 file systems)
支持MP3， WAV，WMA格式歌曲(support MP3, WAV, WMA format songs)
先在sd卡内建立一个名称为“MP3”的文件夹(first, create a folder named "MP3" on the SD card)
然后在文件夹里放入需要播放的歌曲，歌曲格式如下(then, place the songs you want to play in the folder. The song format should be as follows:)
0001+歌曲名，例如歌曲小苹果可以命名如下：0001小苹果(0001+song name, for example, the song "Little Apple" can be named as follows:)
也可以不加歌曲名即0001,其他以此类推0010, 0100, 1000...(you can also omit the song name, just use 0001, and others follow this pattern: 0010, 0100, 1000...)
'''
#使用例程(usage routine)
import smbus
import time
import numpy

class MP3:

    # Global Variables
    address = None
    bus = None

    MP3_PLAY_NUM_ADDR        = 1 #指定曲目播放，0~3000，低位在前，高位在后(designate track playback, 0~3000, with lower digits first and higher digits later)
    MP3_PLAY_ADDR            = 5 #播放(play)
    MP3_PAUSE_ADDR           = 6 #暂停(pause)
    MP3_PREV_ADDR            = 8 #上一曲(last song)
    MP3_NEXT_ADDR            = 9 #下一曲(next song)
    MP3_VOL_VALUE_ADDR       = 12 #指定音量大小0~30(specify volume level 0~30)
    MP3_SINGLE_LOOP_ON_ADDR  = 13 #开启单曲循环,要在播放过程开启才有效(turn on single track loop, must be turned on during playback to be effective)
    MP3_SINGLE_LOOP_OFF_ADDR = 14 #关闭单曲循环(turn off single track loop)

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)        
        
    def play(self):
        self.bus.write_byte(self.address, self.MP3_PLAY_ADDR)
        time.sleep(0.02)
        
    def pause(self):
        self.bus.write_byte(self.address, self.MP3_PAUSE_ADDR)
        time.sleep(0.02)
        
    def prev(self):
        self.bus.write_byte(self.address, self.MP3_PREV_ADDR)
        time.sleep(0.02)
        
    def next(self):
        self.bus.write_byte(self.address, self.MP3_NEXT_ADDR)
        time.sleep(0.02)
        
    def loopOn(self):
        self.bus.write_byte(self.address, self.MP3_SINGLE_LOOP_ON_ADDR)
        time.sleep(0.02)
        
    def loopOff(self):
        self.bus.write_byte(self.address, self.MP3_SINGLE_LOOP_OFF_ADDR)
        time.sleep(0.02)

    def playNum(self, num):
        self.bus.write_word_data(self.address, self.MP3_PLAY_NUM_ADDR, num)
        time.sleep(0.02)
        
    def volume(self, value):
        self.bus.write_word_data(self.address, self.MP3_VOL_VALUE_ADDR, value)
        time.sleep(0.02)
            
if __name__ == "__main__":
    addr = 0x7b #传感器iic地址(sensor I2C address)
    mp3 = MP3(addr)
    mp3.volume(20) #设置音量为20，注意在播放前设置(set the volume to 20, note to set before playback)
    mp3.playNum(23) #播放歌曲3(play song number 3)
    mp3.loopOn() #设置为单曲循环，注意在播放中设置(set to single track loop, note to set during playback)
    time.sleep(20) #延时，验证是否单曲循环中，延时时间应该大于歌曲时间长度(delay to verify if it is in single track loop, delay time should be longer than the song length)
    mp3.loopOff() #关闭循环，如果在歌曲播放中关闭，歌曲会播放完整后才停下(turn off loop, if turned off during song playback, the song will stop only after it finishes)
    mp3.volume(30) 
    mp3.play() #播放(play)
    time.sleep(2)
    mp3.next() #切到下一首，即歌曲序号加1(skip to the next song, i.e., increment the song number by 1)
    time.sleep(2)
    mp3.prev() #切回上一首，即歌曲序号减1(go back to the last song, i.e., decrement the song number by 1)
    time.sleep(2)
    mp3.pause() #暂停(pause)

