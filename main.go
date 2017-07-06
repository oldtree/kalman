package main

import (
	"kalman/kalman"

	"os"

	"os/exec"

	chart "github.com/wcharczuk/go-chart"
	drawing "github.com/wcharczuk/go-chart"
)

func Show() *chart.Chart {
	ts1 := chart.ContinuousSeries{
		Name: "sin(x) x",
		Style: chart.Style{
			Show:      true,
			FontColor: drawing.ColorAlternateBlue,
		},
		XValues: xDimension,
		YValues: realDimotion,
	}

	ts2 := chart.ContinuousSeries{
		Style: chart.Style{
			Show:      true,
			FontColor: drawing.ColorRed,
		},

		XValues: xDimension,
		YValues: yDimension,
	}

	graph := chart.Chart{

		XAxis: chart.XAxis{
			Name:           "x",
			NameStyle:      chart.StyleShow(),
			Style:          chart.StyleShow(),
			ValueFormatter: chart.FloatValueFormatter, //TimeHourValueFormatter,
		},

		YAxis: chart.YAxis{
			Name:      "kalman state value",
			NameStyle: chart.StyleShow(),
			Style:     chart.StyleShow(),
		},

		Series: []chart.Series{
			ts1,
			ts2,
		},
	}
	return &graph
}

var xDimension []float64
var yDimension []float64
var realDimotion []float64

func KalmanOneTest() {
	k := new(kalman.OneDimensionKalman)
	var x float64
	var y float64
	fn := func(xd float64) float64 { return x }
	x = -64
	k.Init(0, 0.1)
	for x <= 64 {
		y = fn(x)

		k.KalmanFilter(float64(y))
		realDimotion = append(realDimotion, y)
		yDimension = append(yDimension, k.State)
		xDimension = append(xDimension, x)

		x = x + 1
	}
	g := Show()
	fi, err := os.Create("./kalman.png")
	if err != nil {
		println(err)
		os.Exit(-1)
	}
	err = g.Render(chart.PNG, fi)
	fi.Sync()
	fi.Close()
	cmd := exec.Command("open", "./kalman.png")
	err = cmd.Run()
	if err != nil {
		println(err)
		os.Exit(-1)
	}
}

func KalmanTwoTest() {

}

func main() {

}
