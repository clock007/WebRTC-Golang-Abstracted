//go:build !jsmill
// +build !jsmill

package main

import (
	"bytes"
	"fmt"
	"strconv"

	"math/rand"

	"path/filepath"

	"github.com/ailumiyana/goav-incr/goav/avcodec"
	"github.com/ailumiyana/goav-incr/goav/avutil"
	"github.com/ailumiyana/goav-incr/goav/swscale"

	//"github.com/mike1808/h264decoder/rgb"
	//""
	//"string"

	//"path"
	//"reflect"

	//"h264writer"

	//"bufio"
	b64 "encoding/base64"
	sg "os/signal"
	"runtime"
	"runtime/debug"

	//"github.com/redis/go-redis/v9"

	"image"
	"time"

	"gocv.io/x/gocv"

	//"encoding/json"

	//"net/url"
	//"io"

	"io"

	//"log"
	"mime/multipart"
	"net/http"

	//"net/http"

	//"os/exec"
	"os"
	"strings"

	//"C"
	"sync"

	//"github.com/aler9/gortsplib/v2/pkg/format"
	//"github.com/pion/rtp/codecs"

	//"github.com/ailumiyana/goav-incr/goav/avcodec"
	//"github.com/ailumiyana/goav-incr/goav/avutil"

	//"h264writer"

	//"github.com/mike1808/h264decoder/decoder"
	//"time"

	//	"decoder"

	"github.com/pion/rtcp"
	//"github.com/pion/rtp"
	"github.com/pion/webrtc/v3"
	"github.com/pion/webrtc/v3/pkg/media"

	//"github.com/wimspaargaren/yolov5"
	"github.com/pion/webrtc/v3/pkg/media/h264writer"
	"github.com/pion/webrtc/v3/pkg/media/oggwriter"

	//"github.com/piovwebrtc/v3/pkg/media/h2644writer"
	//"yolov5"
	//"gocv.io/x/gocv"
	//"github.com/pion/rtp"
	//"image/jpeg"

	//"github.com/bluenviron/gortsplib/v3"
	"github.com/bluenviron/gortsplib/v3/pkg/formats"
	//"github.com/bluenviron/gortsplib/v3/pkg/formats/rtph264"
	//"github.com/bluenviron/gortsplib/v3/pkg/url"

	"github.com/pion/interceptor"
	"github.com/pion/rtp"
)

// var Counter int
var Counts int
var wgU sync.WaitGroup

var (
	yolov5Model   = "/home/nader/data/yolov5/yolov5s.onnx"
	cocoNamesPath = "/home/nader/data/yolov5/coco.names"
)

var uuid string
var channel string

type Decoder struct {
	context   *avcodec.Context
	parser    *avcodec.ParserContext
	frame     *avutil.Frame
	pkt       *avcodec.Packet
	converter *converter
}

type Address_Info struct {
	address         string
	uuid            string
	ChannelCam      int
	StreamStatus    string
	Messages        string
	Nos             string
	file_name_video string
	file_name_audio string
}

// Frame represents decoded frame from H.264/H.265 stream
// Data field will contain bitmap data in the pixel format specified in the decoder
type Frame struct {
	Data                  []byte
	Width, Height, Stride int
}

type converter struct {
	framergb *avutil.Frame
	context  *swscale.Context
	pixFmt   swscale.PixelFormat
}

var message string

func Connector(Addr_Info Address_Info) {
	fmt.Println("runed")
	fmt.Println(Addr_Info.address)
	//Remover(Addr_Info)
	Remove_Files(Addr_Info)

	m := &webrtc.MediaEngine{}

	if err := m.RegisterCodec(webrtc.RTPCodecParameters{
		RTPCodecCapability: webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeH264, ClockRate: 900000, Channels: 0, SDPFmtpLine: "", RTCPFeedback: nil},
		PayloadType:        96,
	}, webrtc.RTPCodecTypeVideo); err != nil {
		panic(err)
	}
	if err := m.RegisterCodec(webrtc.RTPCodecParameters{
		RTPCodecCapability: webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeOpus, ClockRate: 48000, Channels: 0, SDPFmtpLine: "", RTCPFeedback: nil},
		PayloadType:        111,
	}, webrtc.RTPCodecTypeAudio); err != nil {
		panic(err)
	}

	i := &interceptor.Registry{}

	if err := webrtc.RegisterDefaultInterceptors(m, i); err != nil {
		panic(err)
	}

	api := webrtc.NewAPI(webrtc.WithMediaEngine(m), webrtc.WithInterceptorRegistry(i))

	config := webrtc.Configuration{
		ICEServers: []webrtc.ICEServer{
			{
				URLs: []string{"stun:stun.l.google.com:19302"},
			},
		},
	}

	// Create a new RTCPeerConnection
	peerConnection, err := api.NewPeerConnection(config)
	if err != nil {
		panic(err)
	}

	// Allow us to receive 1 audio track, and 1 video track
	if _, err = peerConnection.AddTransceiverFromKind(webrtc.RTPCodecTypeAudio); err != nil {
		panic(err)
	} else if _, err = peerConnection.AddTransceiverFromKind(webrtc.RTPCodecTypeVideo); err != nil {
		panic(err)
	}

	//address_ := strings.Split("", "/")
	//fmt.Println(temp)
	//Nos := strconv.Itoa(No)
	dataChannel, err := peerConnection.CreateDataChannel("data", nil)
	if err != nil {
		panic(err)
	}

	//oggFile, err := oggwriter.New(Addr_Info.file_name_audio, 48000, 2)
	//	if err != nil {
	//	panic(err)
	//}

	//mp4File, err := h264writer.New(Addr_Info.file_name_video)
	//if err != nil {
	//	panic(err)
	//}
	//Addr_Info.file_name_video = file_name_video

	peerConnection.OnTrack(func(track *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {
		fmt.Println("ontrack")
		// Send a PLI on an interval so that the publisher is pushing a keyframe every rtcpPLIInterval
		go func() {
			ticker := time.NewTicker(time.Second * 10)
			for range ticker.C {
				errSend := peerConnection.WriteRTCP([]rtcp.Packet{&rtcp.PictureLossIndication{MediaSSRC: uint32(track.SSRC())}})
				if errSend != nil {
					fmt.Println(errSend)
				}
			}
		}()

		codec := track.Codec()
		if strings.EqualFold(codec.MimeType, webrtc.MimeTypeOpus) {

			fmt.Println("------------ SAve Audio 48 kHz, 2 channels-----)")
			random := rand.Intn(10000)
			file_name_video := strconv.Itoa(random) + ".mp4"
			file_name_audio := strconv.Itoa(random) + ".ogg"
			Addr_Info.file_name_video = file_name_video
			Addr_Info.file_name_audio = file_name_audio

			oggFile, err := oggwriter.New(Addr_Info.file_name_audio, 48000, 2)
			if err != nil {
				panic(err)
			}

			//if err != nil {
			//	panic(err)
			//	}

			ReceiveStream(oggFile, track, dataChannel, Addr_Info)

		} else if strings.EqualFold(codec.MimeType, webrtc.MimeTypeH264) {

			random := rand.Intn(10000)
			file_name_video := strconv.Itoa(random) + ".mp4"
			file_name_audio := strconv.Itoa(random) + ".ogg"
			Addr_Info.file_name_video = file_name_video
			Addr_Info.file_name_audio = file_name_audio

			fmt.Println("H264 Codecs------------------------------")
			mp4File, err := h264writer.New(Addr_Info.file_name_video)
			if err != nil {
				panic(err)
			}
			time.Sleep(2)
			ReceiveStream(mp4File, track, dataChannel, Addr_Info)
		}
	})

	// Set the handler for ICE connection state
	// This will notify you when the peer has connected/disconnected
	peerConnection.OnICEConnectionStateChange(func(connectionState webrtc.ICEConnectionState) {
		fmt.Printf("IC Connection State has changed %s \n", connectionState.String())
		//fmt.Println(Addr_Info.StreamStatus)

		if connectionState.String() == "disconnected" {

			//e := os.Remove(".mp4")
			fmt.Println(Addr_Info.Nos)
			time.Sleep(7 * time.Second)
			//Connector(Addr_Info)
			//fmt.Printf("File does not exist\n");
		}

		if connectionState.String() == "closed" {
			fmt.Println("------------ Closed stae ---------")
		}

		if connectionState == webrtc.ICEConnectionStateConnected {
			fmt.Println("Ctrl+C the remote client to stop the demo")

		} else if connectionState == webrtc.ICEConnectionStateFailed {

			time.Sleep(7 * time.Second)
			fmt.Println("ice failed")
			//Connector(Addr_Info)
			//Connector(Addr_Info)
			//fmt.Printf("File does not exist\n");

			//os.Exit(0)
		}
	})

	peerConnection.OnConnectionStateChange(func(s webrtc.PeerConnectionState) {
		fmt.Printf("Peer Connection State has changed:- %s\n", s.String())

		if s == webrtc.PeerConnectionStateFailed {
			// Wait until PeerConnection has had no network activity for 30 seconds or another failure. It may be reconnected using an ICE Restart.
			// Use webrtc.PeerConnectionStateDisconnected if you are interested in detecting faster timeout.
			// Note that the PeerConnection may come back from PeerConnectionStateDisconnected.
			fmt.Println("Peer Connection has gone to failed exiting")
			time.Sleep(7 * time.Second)
			fmt.Println(Addr_Info.StreamStatus)
			Connector(Addr_Info)
			//os.Exit(0)
		}
	})

	//dataChannel, err := peerConnection.CreateDataChannel("data", nil)
	//if err != nil {
	//	panic(err)
	//}

	dataChannel.OnOpen(func() {
		fmt.Printf("Data channel '%s'-'%d' open. Random messages will now be sent to any connected DataChannels every 5 seconds\n", dataChannel.Label(), dataChannel.ID())

		//for range time.NewTicker(10 * time.Second).C {
		//message := signal.RandSeq(15)
		message := " ****************--------------  Didehnegar DataChannel Stablished --------------------*******"

		//fmt.Printf("Sending '%s'\n", message)

		// Send the message as text
		sendTextErr := dataChannel.SendText(message)
		if sendTextErr != nil {
			fmt.Println("----------  Error Stablihing ---------------")
		}
		//}
	})

	// Register text message handling
	dataChannel.OnMessage(func(msg webrtc.DataChannelMessage) {
		fmt.Printf("Message from DataChannel '%s': '%s'\n", dataChannel.Label(), string(msg.Data))
	})

	offer, err := peerConnection.CreateOffer(nil)

	if err != nil {
		panic(err)
	}

	if err = peerConnection.SetLocalDescription(offer); err != nil {
		panic(err)
	}

	//fmt.Println(offer.SDP)
	fmt.Println("------------------offer encoded---------------")
	sEnc := b64.StdEncoding.EncodeToString([]byte(offer.SDP))
	//sEnc="dj0wDQpvPS0gNDM1MjM3NTUxODU3MzE5MDQyNSAxNjc1ODQ3NzAxIElOIElQNCAwLjAuMC4wDQpzPS0NCnQ9MCAwDQphPWZpbmdlcnByaW50OnNoYS0yNTYgQzg6NUQ6MUU6QzY6Nzg6MDY6RkQ6QzM6MEM6RTQ6NTE6QkE6OEE6Mzk6REQ6Mjc6ODE6Qjk6QjE6MjA6MjQ6Q0E6RDE6NTU6QzI6NUE6QUU6NkI6QzE6RTI6MzY6NTINCmE9ZXh0bWFwLWFsbG93LW1peGVkDQphPWdyb3VwOkJVTkRMRSAwIDEgMg0KbT1hdWRpbyA5IFVEUC9UTFMvUlRQL1NBVlBGIDExMQ0KYz1JTiBJUDQgMC4wLjAuMA0KYT1zZXR1cDphY3RwYXNzDQphPW1pZDowDQphPWljZS11ZnJhZzpYUVZRd2ttSUlUU09pTXFjDQphPWljZS1wd2Q6ZE1iTm12WEJyRWNRQ0JoU2lWUURGbXBTTEtuWnpDQW0NCmE9cnRjcC1tdXgNCmE9cnRjcC1yc2l6ZQ0KYT1ydHBtYXA6MTExIG9wdXMvNDgwMDANCmE9cnRjcC1mYjoxMTEgdHJhbnNwb3J0LWNjIA0KYT1leHRtYXA6MSBodHRwOi8vd3d3LmlldGYub3JnL2lkL2RyYWZ0LWhvbG1lci1ybWNhdC10cmFuc3BvcnQtd2lkZS1jYy1leHRlbnNpb25zLTAxDQphPXNzcmM6MjMxMDIzNTQ3NiBjbmFtZTpWa2Zrd3J0ZWlocEttVUdsDQphPXNzcmM6MjMxMDIzNTQ3NiBtc2lkOlZrZmt3cnRlaWhwS21VR2wgQ0xvamNqUUhMeFVWRERHSg0KYT1zc3JjOjIzMTAyMzU0NzYgbXNsYWJlbDpWa2Zrd3J0ZWlocEttVUdsDQphPXNzcmM6MjMxMDIzNTQ3NiBsYWJlbDpDTG9qY2pRSEx4VVZEREdKDQphPW1zaWQ6Vmtma3dydGVpaHBLbVVHbCBDTG9qY2pRSEx4VVZEREdKDQphPXNlbmRyZWN2DQptPXZpZGVvIDkgVURQL1RMUy9SVFAvU0FWUEYgOTYNCmM9SU4gSVA0IDAuMC4wLjANCmE9c2V0dXA6YWN0cGFzcw0KYT1taWQ6MQ0KYT1pY2UtdWZyYWc6WFFWUXdrbUlJVFNPaU1xYw0KYT1pY2UtcHdkOmRNYk5tdlhCckVjUUNCaFNpVlFERm1wU0xLblp6Q0FtDQphPXJ0Y3AtbXV4DQphPXJ0Y3AtcnNpemUNCmE9cnRwbWFwOjk2IEgyNjQvOTAwMDANCmE9cnRjcC1mYjo5NiBuYWNrIA0KYT1ydGNwLWZiOjk2IG5hY2sgcGxpDQphPXJ0Y3AtZmI6OTYgdHJhbnNwb3J0LWNjIA0KYT1leHRtYXA6MSBodHRwOi8vd3d3LmlldGYub3JnL2lkL2RyYWZ0LWhvbG1lci1ybWNhdC10cmFuc3BvcnQtd2lkZS1jYy1leHRlbnNpb25zLTAxDQphPXNzcmM6MjMyNDY2MDUxOCBjbmFtZTpXR1JVeVJ3ekFTZmhKb0NaDQphPXNzcmM6MjMyNDY2MDUxOCBtc2lkOldHUlV5Und6QVNmaEpvQ1ogYVVoUGJrdFNpYlpWaEtGUw0KYT1zc3JjOjIzMjQ2NjA1MTggbXNsYWJlbDpXR1JVeVJ3ekFTZmhKb0NaDQphPXNzcmM6MjMyNDY2MDUxOCBsYWJlbDphVWhQYmt0U2liWlZoS0ZTDQphPW1zaWQ6V0dSVXlSd3pBU2ZoSm9DWiBhVWhQYmt0U2liWlZoS0ZTDQphPXNlbmRyZWN2DQptPWFwcGxpY2F0aW9uIDkgVURQL0RUTFMvU0NUUCB3ZWJydGMtZGF0YWNoYW5uZWwNCmM9SU4gSVA0IDAuMC4wLjANCmE9c2V0dXA6YWN0cGFzcw0KYT1taWQ6Mg0KYT1zZW5kcmVjdg0KYT1zY3RwLXBvcnQ6NTAwMA0KYT1pY2UtdWZyYWc6WFFWUXdrbUlJVFNPaU1xYw0KYT1pY2UtcHdkOmRNYk5tdlhCckVjUUNCaFNpVlFERm1wU0xLblp6Q0FtDQo="
	fmt.Println(sEnc)

	contType, reader, err := createReqBody(sEnc)

	//uuid := "bdee3e92-b817-11ed-9db7-f1658f21cba9"

	req, err := http.NewRequest("POST", Addr_Info.address, reader)
	client := &http.Client{}

	req.Header.Add("Content-Type", contType)
	resp, err := client.Do(req)

	ss := 0
	defer func(ss int) {
		if panicInfo := recover(); panicInfo != nil {
			fmt.Printf("%v, %s", panicInfo, string(debug.Stack()))
			fmt.Println(Addr_Info.StreamStatus)

			time.Sleep(5 * time.Second)
			Connector(Addr_Info)

		}
	}(ss)

	defer resp.Body.Close()
	bodyText, err := io.ReadAll(resp.Body)
	if err != nil {
		fmt.Println(err)
	}

	sdp_temp := string(bodyText)

	sDec, err := b64.StdEncoding.DecodeString(sdp_temp)
	Dec := string(sDec)

	answer2 := webrtc.SessionDescription{}
	answer2.SDP = Dec

	err = peerConnection.SetRemoteDescription(answer2)

}

//---------------------------------------------- end of Connector ----------------------------

func createReqBody(sen string) (string, io.Reader, error) {
	//var err error

	buf := new(bytes.Buffer)
	bw := multipart.NewWriter(buf) // body writer

	// text part1
	p1w, _ := bw.CreateFormField("data")

	p1w.Write([]byte(sen))
	fmt.Println("---offer----")
	fmt.Printf(sen)
	//var j media.Writer

	// text part2
	//p2w, _ := bw.CreateFormField("age")
	//p2w.Write([]byte("15"))

	// file part1
	//_, fileName := filepath.Split(filePath)
	//fw1, _ := bw.CreateFormFile("file1", fileName)
	//io.Copy(fw1, f)

	bw.Close() //write the tail boundry
	return bw.FormDataContentType(), buf, nil
}

func ReceiveStream(i media.Writer, track *webrtc.TrackRemote, DChannel *webrtc.DataChannel, Addr_Info Address_Info) {

	var forma formats.H264
	// medi := medias.FindFormat(&forma)
	// if medi == nil {
	// panic("media not found")
	// }
	//
	// setup RTP/H264->H264 decoder
	fmt.Println("decoder maked1")
	rtpDec := forma.CreateDecoder()
	h264RawDec, err := newH264Decoder()
	message = ""
	// h264RawDec.close()
	// h264RawDec, err = newH264Decoder()
	// fmt.Println("decoder maked")
	//
	//	if err != nil {
	//		fmt.Println(err)
	//	}
	//
	// var wgU sync.WaitGroup
	// Counts = 0
	// Counter = 0
	// Counter := make(chan int,1)
	// Counter <-0

	fmt.Println("************************* Begin Routine********************************************")

	//Switch:=0
	// Gracefully close the net when the program is done
	//yolonet, err := yolov5.NewNet(yolov5Model, cocoNamesPath)
	window := gocv.NewWindow(string(Addr_Info.Nos))

	//fmt.Println(Counter,Counts)4
	//defer func() {
	//if err := i.Close(); err != nil {
	//panic(err)
	//fmt.Println(err)
	//}
	//}()

	stream, err := os.Open(string(Addr_Info.file_name_video))

	// stream, err = os.Open("in.mp4")
	fmt.Println(err)
	// if err == nil {
	// d, err := decoder.New(decoder.PixelFormatBGR)
	if err != nil {
		fmt.Println("error Creating Decoders")
	}

	var rtpPacket *rtp.Packet

	var Counter_Existence int
	var Counter int
	//lock := sync.RWMutex{}
	rtpPacket, _, err = track.ReadRTP()
	_, _, err = rtpDec.Decode(rtpPacket)
	Addr_Info.StreamStatus = "********** running *********"
	//var wgs sync.WaitGroup
	//track.SetReadDeadline(time.Now().Add(5000 * time.Millisecond))

	for {
		//fmt.Println("------------- re run -------------------")

		if Counter_Existence > 300 {
			fmt.Println("check-------------------file")
			if _, err := os.Stat(Addr_Info.file_name_video); err == nil {
				fmt.Println("exists file")
			} else {
				fmt.Println("****file doesnot exist and out from routine****")
				stream.Close()
				window.Close()
				runtime.GC()
				rtpPacket = nil
				///Addr_Info.StreamStatus = "Not Running"
				debug.FreeOSMemory()
				break
				//fmt.Printf("File does not exist\n");
			}
			Counter_Existence = 0
		}

		//fmt.Println("re run")

		ss := 0
		//fmt.Println("reading Packet")

		//defer func(ss int) {672

		//	if panicInfo := recover(); panicInfo != nil {
		//fmt.Printf("%v, %s", panicInfo, string(debug.Stack()))
		//		fmt.Println("----------error reading-------------------0")

		//time.\Sleep(1 * time.Second)
		//continue
		//Connector(address)
		//continue

		//	}
		//}(ss)
		//fmt.Println("try to read")
		rtpPacket, _, err = track.ReadRTP()
		time.Sleep(1000)
		if err != nil {
			fmt.Println(err)
			Counter_Existence = Counter_Existence + 1
			continue
		}
		//fmt.Println("can not read rtp packet")
		//if _, err := os.Stat(string(Addr_Info.Nos) + "in.mp4"); err == nil {
		//	fmt.Println("exists file")
		//	} else {
		//	fmt.Println("file doesnot exist and out from routine")
		//	stream.Close()
		//	window.Close()
		//	runtime.GC()
		//	rtpPacket = nil
		///Addr_Info.StreamStatus = "Not Running"
		//	debug.FreeOSMemory()
		//	break
		//fmt.Printf("File does not exist\n");
		//	}
		//Counts = Counts + 1
		//fmt.Println(Addr_Info.StreamStatus)
		//	continue

		//if Counts > 1000 {
		//break
		//}
		//continue
		//}

		//buff := make([]byte, 2048)

		//message :="******************         "+ string(Nos)+ "  ********************************"
		//fmt.Printf("Sending '%s'\n", message)

		// Send the message as text
		//sendTextErr := DChannel.SendText(message)
		//fmt.Println(sendTextErr)

		//}

		Counter = Counter + 1
		Counter_Existence = Counter_Existence + 1
		// if (Counter ==50 ){
		//               message ="******************         "+ string(Nos)+ "  ********************************"

		//fmt.Printf("Sending '%s'\n", message)

		// Send the message as text
		//sendTextErr := DChannel.SendText(message)
		//fmt.Println(sendTextErr)
		//Counter=0

		//rtpDec = forma.CreateDecoder()
		//Counter=0
		//}

		//if (Counter > 1000) {
		// window.Close()
		//window = gocv.NewWindow(Nos)
		//Counter=0
		// }

		//if err := i.WriteRTP(rtpPacket); err != nil {
		//panic(err)
		//	fmt.Println(err)
		//continue
		//}

		//fmt.Println(err)
		nalus, _, err := rtpDec.Decode(rtpPacket)
		time.Sleep(10000)
		//if (Counter >10){
		//nalus, _, err := rtpDec.Decode(rtpPacket)
		if err != nil {
			fmt.Println(err)
			//continue
		}
		fmt.Println(err)
		//if (err==nil){
		//Counter=0}
		//message="--------------new-----------------"
		//message :="************************   "+ string(Nos)+ "   ****************************"
		//sendTextErr := DChannel.SendText(message)
		//fmt.Println(sendTextErr)

		//Counter=0
		//lock.RLock()

		//nalus, _, err := rtpDec.Decode(rtpPacket)
		//lock.RUnlock()
		//if (Counter <100){
		//continue
		//}
		//if (err!=nil){
		//  continue}

		fmt.Println("------------- Ddecoding System  --------------")

		if err == nil {
			rtpPacket = nil
			for _, nalu := range nalus {
				fmt.Println("decode frames")
				// convert NALUs into RGBA frames
				defer func(ss int) {
					if panicInfo := recover(); panicInfo != nil {

						//rtpDec = forma.CreateDecoder()
						//fmt.Printf("%v, %s", panicInfo, string(debug.Stack()))
						fmt.Println("error decoding-------------------2")

						//time.Sleep(5*time.Second)
						//Connector(address)

					}
				}(ss)

				//frame:=nil
				//if (Counter > 50 ){
				// type  Image.img
				frame, err := h264RawDec.decode(nalu)
				time.Sleep(1000)

				if err != nil {
					continue
				}
				if frame == nil {
					continue
					nalu = nil
				} // nil frame
				//fmt.Println("decoded frame with size %v ", frame.Bounds().Max)
				//fmt.Println(err)
				//lock.RLock()
				//wgs.Add(1)

				//defer wgs.Done()
				if Counter > 150 {
					fm, err := ToRGB8(frame)
					if err != nil {
						continue
					}
					Addr_Info.Messages = "************************   " + string(Addr_Info.Nos) + "   ****************************"

					sendTextErr := DChannel.SendText(Addr_Info.Messages)
					fmt.Println(sendTextErr)
					fmt.Println(err)
					//if (err!=nil){
					//continue}
					//func ToRGB8(img image.Image) (gocv.Mat, error) {

					//bytes=nil

					//lock.RUnlock()
					//fm, err := gocv.NewMatFromBytes(frame.Height, frame.Width, gocv.MatTypeCV8UC3, frame.Data)
					//bounds := frame.Bounds()
					//fm, err := gocv.NewMatFromBytes(bounds.Dx(),bounds.Dy(),1,buff)
					//detections, err := yolonet.GetDetections(mg)
					//fmt.Println(detections)
					//yolov5.DrawDetections(&fm, detections)
					//message="--------------new-----------------"
					//fmt.Println(err)
					//sendTextErr := DChannel.SendText(message)
					//fmt.Println(sendTextErr)

					//Counter=0
					window.IMShow(fm)
					window.WaitKey(1)
					Counter = 0
					fm.Close()
					frame = nil
					//runtime.GC()
					//debug.FreeOSMemory()

				}
				//fm.Close()

				//runtime.GC()
				//debug.FreeOSMemory()

			} // for nalue

		} // if err= nil

		//

		//track.SetReadDeadline(time.Now().Add(5000 * time.Millisecond))

	} // for
	//go ReceiveStream(i,track, Nos,DChannel)
	runtime.GC()
	///Addr_Info.StreamStatus = "Not Running"
	debug.FreeOSMemory()
	//defer wgU.Done()
	stream.Close()
	window.Close()

}

func main() {
	uuid = "ac227710-052d-11ee-8b82-0242ac160002"
	uuid = "97cd9250-3b41-11ee-bec1-50ebf6ba5f1d/"
	channel = "1"

	//address := "http://192.168.1.12:8083/stream/" + string(uuid) + "/channel/" + string(channel) + "/webrtc"
	//fmt.Println(address)

	fmt.Println("==========================================================")

	//var Addresses = [...]string{"http://192.168.1.86:8083/stream/771d86e4-f94e-11ed-a900-0242ac130002/channel/1/webrtc",
	//	"http://192.168.1.86:8083/stream/771ea826-f94e-11ed-a900-0242ac130002/channel/1/webrtc",
	//	"http://192.168.1.86:8083/stream/771fe70e-f94e-11ed-a900-0242ac130002/channel/1/webrtc",
	//	"http://192.168.1.86:8083/stream/7720874a-f94e-11ed-a900-0242ac130002/channel/1/webrtc"}
	//        "http://192.168.1.12:8083/stream/"+string(uuid)+"/channel/1/webrtc"}

	var Addresses = [...]string{"http://192.168.1.85:8000/stream/abce203e-39da-11ee-9e70-50ebf6ba5f1d/channel/0/webrtc",
		"http://192.168.1.85:8000/stream/97cd9250-3b41-11ee-bec1-50ebf6ba5f1d/channel/0/webrtc",
		"http://192.168.1.85:8000/stream/ef8831e8-0cf8-11ee-80e4-0242ac120002/channel/1/webrtc",
		"http://192.168.1.85:8000/stream/ef88563c-0cf8-11ee-80e4-0242ac120002/channel/1/webrtc",
		"http://192.168.1.85:8000/stream/ef88563c-0cf8-11ee-80e4-0242ac120002/channel/1/webrtc",
		"http://192.168.1.85:8000/stream/ef88563c-0cf8-11ee-80e4-0242ac120002/channel/1/webrtc",
		"http://192.168.1.85:8000/stream/ef888080-0cf8-11ee-80e4-0242ac120002/channel/1/webrtc"}

	//All_Data := [...]Address_info{}

	//	for {
	No := 0

	for _, addr := range Addresses {
		//uuids= strings.Split(addr,"/")
		Addr_Info := Address_Info{}
		Addr_Info.address = addr
		Addr_Info.uuid = uuid
		Addr_Info.Nos = strconv.Itoa(No)
		Addr_Info.StreamStatus = "not Running"

		//fmt.Println(aad[4])
		//fmt.Println("******************** begin-------------------------------*********")
		wgU.Add(1)
		random := rand.Intn(10000)
		file_name_video := strconv.Itoa(random) + ".mp4"
		file_name_audio := strconv.Itoa(random) + ".ogg"
		Addr_Info.file_name_video = file_name_video
		Addr_Info.file_name_audio = file_name_audio

		Connector(Addr_Info)

		No = No + 1
		//fmt.Println(addr)

		runtime.GC()
		debug.FreeOSMemory()
	}
	wgU.Wait()
	//	}

	closed := make(chan os.Signal, 1)
	sg.Notify(closed, os.Interrupt)

	<-closed
	//fmt.Println("closed")
	//if closeErr := oggFile.Close(); closeErr != nil {
	//	panic(closeErr)
	//}
	//fmt.Println(err)

	//if closeErr := mp4File.Close(); closeErr != nil {
	//	panic(closeErr)

	//}
	//fmt.Println(err)

	//if err := peerConnection.Close(); err != nil {
	//	panic(err)
	//	if closeErr := oggFile.Close(); closeErr != nil {
	//		panic(closeErr)
	//	}

	//	if closeErr := mp4File.Close(); closeErr != nil {
	//		panic(closeErr)
	//		fmt.Println("closed")
	//	}
	//}

}

func ToRGB8(img image.Image) (gocv.Mat, error) {
	bounds := img.Bounds()
	x := bounds.Dx()
	y := bounds.Dy()
	bytes := make([]byte, 0, x*y*3)

	//don't get surprised of reversed order everywhere below
	for j := bounds.Min.Y; j < bounds.Max.Y; j++ {
		for i := bounds.Min.X; i < bounds.Max.X; i++ {
			r, g, b, _ := img.At(i, j).RGBA()
			bytes = append(bytes, byte(b>>8), byte(g>>8), byte(r>>8))
		}
	}
	//bytes=nil
	return gocv.NewMatFromBytes(y, x, gocv.MatTypeCV8UC3, bytes)
}

func WalkMatch(root, pattern string) ([]string, error) {
	var matches []string
	err := filepath.Walk(root, func(path string, info os.FileInfo, err error) error {
		if err != nil {
			return err
		}
		if info.IsDir() {
			return nil
		}
		if matched, err := filepath.Match(pattern, filepath.Base(path)); err != nil {
			return err
		} else if matched {
			matches = append(matches, path)
		}
		return nil
	})
	if err != nil {
		return nil, err
	}
	return matches, nil
}

func Remover(Addr_Info Address_Info) {
	files, err := WalkMatch("./", "*.mp4")
	fmt.Println(err)

	for _, file := range files {
		fmt.Println(file)
		if filepath.Ext(file) == ".mp4" {
			os.Remove(file)
		}
	}

	files, err = WalkMatch("./", "*.ogg")
	fmt.Println(err)

	for _, file := range files {
		fmt.Println(file)
		if filepath.Ext(file) == ".mp4" || filepath.Ext(file) == ".ogg" {
			os.Remove(file)
		}
	}

}

func Remove_Files(Addr_Info Address_Info) {
	if _, err := os.Stat(Addr_Info.file_name_video); err == nil {
		fmt.Printf("File exists\n")
		e := os.Remove(Addr_Info.file_name_video)
		if e != nil {
			fmt.Println(e)
		}

	} else {
		fmt.Printf("File does not exist\n")
	}

	if _, err := os.Stat(Addr_Info.file_name_audio); err == nil {
		fmt.Printf("File exists\n")
		e := os.Remove(Addr_Info.file_name_video)
		if e != nil {
			fmt.Println(e)
		}

	} else {
		fmt.Printf("File does not exist\n")
	}
}
