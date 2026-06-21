#!/usr/bin/perl
use strict; use warnings;

# ---- CRC-16/XMODEM (matches firmware crc16.c) ----
my @tab;
for my $i (0..255){ my $c=($i<<8)&0xFFFF; for(1..8){ $c = ($c&0x8000)?((($c<<1)^0x1021)&0xFFFF):(($c<<1)&0xFFFF);} $tab[$i]=$c; }
sub crc16 { my @b=@_; my $crc=0; for my $x(@b){ $crc = (($crc<<8)^$tab[(($crc>>8)^$x)&0xFF])&0xFFFF; } return $crc; }
sub f32 { return unpack("C4", pack("f<", $_[0])); }

sub frame_hex {
  my ($cmd, @pay) = @_;
  my @data = ($cmd, @pay);
  my $len  = scalar @data;
  my $lh=($len>>8)&0xFF; my $ll=$len&0xFF;
  my $hc=(0xA5+0x5A+$lh+$ll)&0xFF;
  my $crc=crc16(@data); my $cl=$crc&0xFF; my $ch=($crc>>8)&0xFF;
  my @all = (0xA5,0x5A,$lh,$ll,$hc,@data,$cl,$ch);
  return join("", map { sprintf("%02X",$_) } @all);
}

sub js { my $s=shift; $s =~ s/\\/\\\\/g; $s =~ s/"/\\"/g; $s =~ s/\r/\\r/g; $s =~ s/\n/\\n/g; $s =~ s/\t/\\t/g; return '"'.$s.'"'; }

my $pf = "w:/Items/JointMotor/SW/jointmotor/User/Protocol/serial_studio/frame_parser.js";
open(my $fh, "<", $pf) or die "open $pf: $!";
local $/; my $parser = <$fh>; close($fh);

sub ds {
  my (%o)=@_;
  my $graph = $o{graph} ? "true":"false";
  my $idx=$o{index}; my $pmin=$o{plotMin}//0; my $pmax=$o{plotMax}//0;
  my $wmin=$o{widgetMin}//0; my $wmax=$o{widgetMax}//0;
  my @L;
  push @L, '                {';
  push @L, '                    "alarmEnabled": false,';
  push @L, '                    "alarmHigh": 0,';
  push @L, '                    "alarmLow": 0,';
  push @L, '                    "fft": false,';
  push @L, '                    "fftMax": 0,';
  push @L, '                    "fftMin": 0,';
  push @L, '                    "fftSamples": 1024,';
  push @L, '                    "fftSamplingRate": -1,';
  push @L, '                    "graph": '.$graph.',';
  push @L, '                    "index": '.$idx.',';
  push @L, '                    "led": false,';
  push @L, '                    "ledHigh": 1,';
  push @L, '                    "log": false,';
  push @L, '                    "overviewDisplay": false,';
  push @L, '                    "plotMax": '.$pmax.',';
  push @L, '                    "plotMin": '.$pmin.',';
  push @L, '                    "title": '.js($o{title}).',';
  push @L, '                    "units": '.js($o{units}//"").',';
  push @L, '                    "value": "--.--",';
  push @L, '                    "widget": '.js($o{widget}//"").',';
  push @L, '                    "widgetMax": '.$wmax.',';
  push @L, '                    "widgetMin": '.$wmin.',';
  push @L, '                    "xAxis": -1';
  push @L, '                }';
  return join("\n", @L);
}

sub act {
  my ($title,$hex,$icon)=@_;
  $icon //= "Play Property";
  my @L;
  push @L, '        {';
  push @L, '            "autoExecuteOnConnect": false,';
  push @L, '            "binary": true,';
  push @L, '            "eol": "",';
  push @L, '            "icon": '.js($icon).',';
  push @L, '            "repeatCount": 3,';
  push @L, '            "sourceId": 0,';
  push @L, '            "timerIntervalMs": 100,';
  push @L, '            "timerMode": 0,';
  push @L, '            "title": '.js($title).',';
  push @L, '            "txData": '.js($hex).',';
  push @L, '            "txEncoding": 0';
  push @L, '        }';
  return join("\n", @L);
}

my @realtime = (
  ds(index=>1,title=>"位置 Pos",    units=>"rad",  widget=>"gauge",graph=>1,plotMin=>-12.5,plotMax=>12.5,widgetMin=>-12.5,widgetMax=>12.5),
  ds(index=>2,title=>"速度 Vel",    units=>"rad/s",widget=>"gauge",graph=>1,plotMin=>-65,plotMax=>65,widgetMin=>-65,widgetMax=>65),
  ds(index=>3,title=>"力矩 Torque", units=>"Nm",   widget=>"gauge",graph=>1,plotMin=>-50,plotMax=>50,widgetMin=>-50,widgetMax=>50),
);
my @health = (
  ds(index=>4,title=>"温度 Temp",     units=>"C", widget=>"bar",graph=>1,plotMin=>0,plotMax=>120,widgetMin=>0,widgetMax=>120),
  ds(index=>5,title=>"母线电压 Vbus", units=>"V", widget=>"bar",graph=>1,plotMin=>0,plotMax=>60,widgetMin=>0,widgetMax=>60),
  ds(index=>6,title=>"故障码 Err",    units=>"",  widget=>"",   graph=>0),
);

my @actions = (
  act("上使能 Enable",       frame_hex(0x04)),
  act("下使能 Disable",      frame_hex(0x05)),
  act("停止运行 Stop",       frame_hex(0x06)),
  act("位置保持 Hold",       frame_hex(0x01)),
  act("紧急停止 E-Stop",     frame_hex(0x03)),
  act("清除故障 ClearFault", frame_hex(0xB0)),
  act("保存配置 SaveCfg",    frame_hex(0xB3)),
  act("读实时反馈 0xC0",     frame_hex(0xC0)),
  act("读电机状态 0xC1",     frame_hex(0xC1)),
  act("读故障码 0xC8",       frame_hex(0xC8)),
  act("读设备信息 0xD0",     frame_hex(0xD0)),
  act("回零 Homing(法0)",    frame_hex(0x54, 0)),
  act("位置环 -> 0 rad",     frame_hex(0x15, f32(0.0))),
  act("位置环 -> 1.0 rad",   frame_hex(0x15, f32(1.0))),
  act("位置环 -> -1.0 rad",  frame_hex(0x15, f32(-1.0))),
  act("速度环 -> 0 rad/s",   frame_hex(0x14, f32(0.0))),
  act("速度环 -> 5 rad/s",   frame_hex(0x14, f32(5.0))),
  act("力矩环 -> 0.5 Nm",    frame_hex(0x12, f32(0.5))),
  act("开始日志 100Hz",      frame_hex(0xB5, 100,0, 0xFF,0xFF,0xFF,0xFF)),
  act("停止日志",            frame_hex(0xB6)),
);

my $rt = join(",\n", @realtime);
my $hl = join(",\n", @health);
my $ac = join(",\n", @actions);

my @P;
push @P, '{';
push @P, '    "actions": [';
push @P, $ac;
push @P, '    ],';
push @P, '    "checksum": "",';
push @P, '    "decoder": 3,';
push @P, '    "frameDetection": 2,';
push @P, '    "frameEnd": "",';
push @P, '    "frameParser": '.js($parser).',';
push @P, '    "frameStart": "",';
push @P, '    "groups": [';
push @P, '        {';
push @P, '            "datasets": [';
push @P, $rt;
push @P, '            ],';
push @P, '            "title": "实时反馈 (0xC0)",';
push @P, '            "widget": ""';
push @P, '        },';
push @P, '        {';
push @P, '            "datasets": [';
push @P, $hl;
push @P, '            ],';
push @P, '            "title": "健康状态 (0xC0)",';
push @P, '            "widget": ""';
push @P, '        }';
push @P, '    ],';
push @P, '    "hexadecimalDelimiters": false,';
push @P, '    "title": "关节电机 JointMotor UART"';
push @P, '}';
my $out = join("\n", @P)."\n";

my $dest = "w:/Items/JointMotor/SW/jointmotor/User/Protocol/serial_studio/jointmotor_uart.ssproj";
open(my $o, ">", $dest) or die "write $dest: $!";
binmode($o);
print $o $out; close($o);
print "WROTE $dest (", length($out), " bytes)\n";
print "Enable     = ", frame_hex(0x04), "\n";
print "Pos 1.0rad = ", frame_hex(0x15, f32(1.0)), "\n";
print "ReadFeedbk = ", frame_hex(0xC0), "\n";
