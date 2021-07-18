const updateRateSecs = 0.01
const originX = 519
const originY = 719
const inchesToPixels = 1291 / 323.25 // Field height (px) divided by field height (inches)
const robotWidthPixels = 30 * inchesToPixels
const trailLengthSecs = 15

var filePicker = document.getElementById("filePicker")
var field = document.getElementById("field")
var fieldContext = field.getContext("2d")
var fieldImg = document.getElementById("fieldImg")
var playButton = document.getElementById("play")
var pauseButton = document.getElementById("pause")
var timestamp = document.getElementById("timestamp")
var timeline = document.getElementById("timeline")
var markerContainer = document.getElementById("markerContainer")

var playing = false
var resetOnPlay = false
var canvasReady = false
var fileReady = false
var alliance = "Blue"
var fileData = []

// --- Play/pause functions ---
function playpause() {
  playing = !playing
  playButton.hidden = playing
  pauseButton.hidden = !playing
  if (playing) {
    lastUpdate = new Date().getTime()
  }
  if (playing && resetOnPlay) {
    resetOnPlay = false
    timeline.value = timeline.min
  }
}

document.body.onkeypress = function (event) {
  if (event.keyCode == 32) {
    playpause()
  }
}

// --- File reading functions ---
filePicker.addEventListener("change", function () {
  var reader = new FileReader()
  reader.onload = function () {
    fileData = []

    // Decode CSV
    var lines = reader.result.split("\n")
    for (let i in lines) {
      if (i == 0) {
        alliance = lines[i]
      } else if (i > 1) {
        var values = lines[i].split(",")
        if (values.length == 5) {
          fileData.push({
            "timestamp": +(values[0]),
            "enabled": values[1] == "1",
            "x": +(values[2]),
            "y": +(values[3]),
            "rotation": +(values[4])
          })
        }
      }
    }

    // Update min and max
    if (playing) {
      playpause()
    }
    timeline.min = fileData[0].timestamp
    timeline.value = timeline.min
    timeline.max = fileData[fileData.length - 1].timestamp

    // Find enabled ranges
    var enabledRanges = []
    var lastEnabled = false
    for (let i in fileData) {
      if (lastEnabled) {
        if (!fileData[i].enabled) {
          lastEnabled = false
          enabledRanges[enabledRanges.length - 1].push(fileData[i - 1].timestamp)
        }
      } else {
        if (fileData[i].enabled) {
          lastEnabled = true
          enabledRanges.push([fileData[i].timestamp])
        }
      }
    }
    if (enabledRanges.length > 0) {
      if (enabledRanges[enabledRanges.length - 1].length < 2) {
        enabledRanges[enabledRanges.length - 1].push(timeline.max)
      }
    }

    // Render enabled ranges
    while (markerContainer.firstChild) {
      markerContainer.removeChild(markerContainer.lastChild)
    }
    for (let i in enabledRanges) {
      var startPercent = ((enabledRanges[i][0] - timeline.min) / (timeline.max - timeline.min)) * 100
      var durationPercent = ((enabledRanges[i][1] - enabledRanges[i][0]) / (timeline.max - timeline.min)) * 100

      var marker = document.createElement("div")
      marker.classList.add("timeline-marker")
      marker.style.backgroundColor = "lightgreen"
      marker.style.left = startPercent.toString() + "%"
      marker.style.width = durationPercent.toString() + "%"
      markerContainer.appendChild(marker)
    }

    fileReady = true
  }
  reader.readAsText(this.files[0])
})

function getPose(timestamp) {
  var index = 0
  while (index < fileData.length - 1) {
    if (fileData[index].timestamp > timestamp) {
      break
    } else {
      index++
    }
  }
  var next = fileData[index]
  var prev = fileData[index - 1]
  var timePortion = (timestamp - prev.timestamp) / (next.timestamp - prev.timestamp)
  var rotationDiff = next.rotation - prev.rotation;
  while (rotationDiff > 180) {
    rotationDiff -= 360
  }
  while (rotationDiff < -180) {
    rotationDiff += 360
  }
  return {
    "enabled": prev.enabled,
    "x": ((next.x - prev.x) * timePortion) + prev.x,
    "y": ((next.y - prev.y) * timePortion) + prev.y,
    "rotation": (rotationDiff * timePortion) + prev.rotation
  }
}

// --- Canvas functions ---
function setupCanvas() {
  field.height = fieldImg.height
  field.width = fieldImg.width
  canvasReady = true
}

function transform(x, y, angle, centerX, centerY) {
  var length = Math.sqrt((x * x) + (y * y))
  var newAngle = Math.atan2(y, x) + (angle * (Math.PI / 180))
  return [
    centerX + (Math.cos(newAngle) * length),
    centerY - (Math.sin(newAngle) * length)
  ]
}

var lastUpdate = 0
function update() {
  // Change time if playing
  if (playing) {
    var time = new Date().getTime()
    var newValue = +(timeline.value) + ((time - lastUpdate) / 1000)
    lastUpdate = time
    timeline.value = newValue
    if (newValue >= +(timeline.max)) {
      playpause()
      resetOnPlay = true
    }
  }

  // Update timestamp
  var time = new Number(timeline.value)
  if (time < 10) {
    timestamp.innerHTML = time.toFixed(3)
  } else if (time < 100) {
    timestamp.innerHTML = time.toFixed(2)
  } else if (time < 1000) {
    timestamp.innerHTML = time.toFixed(1)
  } else {
    timestamp.innerHTML = time.toFixed(0)
  }

  // Render canvas
  if (canvasReady) {
    var isRed = alliance == "Red"
    fieldContext.clearRect(0, 0, field.width, field.height)
    if (isRed) {
      fieldContext.translate(field.width, field.height)
      fieldContext.scale(-1, -1)
      fieldContext.drawImage(fieldImg, 0, 0)
      fieldContext.setTransform(1, 0, 0, 1, 0, 0)
    } else {
      fieldContext.drawImage(fieldImg, 0, 0)
    }

    if (fileReady) {
      // Render trails
      fieldContext.strokeStyle = "#aaaaaa"
      fieldContext.lineWidth = 1 * inchesToPixels
      fieldContext.lineCap = "round"
      fieldContext.lineJoin = "round"

      var trailTime = +(timeline.value) - trailLengthSecs
      trailTime = trailTime < +(timeline.min) ? +(timeline.min) : trailTime
      var trailEnd = +(timeline.value) + trailLengthSecs
      trailEnd = trailEnd > +(timeline.max) ? +(timeline.max) : trailEnd

      var prevPoint = null
      while (true) {
        var pose = getPose(trailTime)
        var x = originX + (pose.x * inchesToPixels)
        var y = originY - (pose.y * inchesToPixels)
        if (prevPoint == null) {
          fieldContext.beginPath()
          fieldContext.moveTo(x, y)
        } else {
          var dist = Math.sqrt((x - prevPoint[0]) * (x - prevPoint[0]) + (y - prevPoint[1]) * (y - prevPoint[1]))
          if (dist > 36 * inchesToPixels) {
            fieldContext.stroke()
            fieldContext.beginPath()
            fieldContext.moveTo(x, y)
          } else {
            fieldContext.lineTo(x, y)
          }
        }
        prevPoint = [x, y]

        if (trailTime == trailEnd) {
          break
        }
        trailTime += updateRateSecs * 5
        if (trailTime > trailEnd) {
          trailTime = trailEnd
        }
      }
      fieldContext.stroke()

      // Render robot
      var pose = getPose(timeline.value)
      var centerX = originX + (pose.x * inchesToPixels)
      var centerY = originY - (pose.y * inchesToPixels)

      fieldContext.fillStyle = "#222222"
      fieldContext.strokeStyle = isRed ? "red" : "blue"
      fieldContext.lineWidth = 3 * inchesToPixels
      var fl = transform(robotWidthPixels / 2, robotWidthPixels / 2, pose.rotation, centerX, centerY)
      var fr = transform(robotWidthPixels / 2, -robotWidthPixels / 2, pose.rotation, centerX, centerY)
      var br = transform(-robotWidthPixels / 2, -robotWidthPixels / 2, pose.rotation, centerX, centerY)
      var bl = transform(-robotWidthPixels / 2, robotWidthPixels / 2, pose.rotation, centerX, centerY)
      fieldContext.beginPath()
      fieldContext.moveTo(fl[0], fl[1])
      fieldContext.lineTo(fr[0], fr[1])
      fieldContext.lineTo(br[0], br[1])
      fieldContext.lineTo(bl[0], bl[1])
      fieldContext.closePath()
      fieldContext.fill()
      fieldContext.stroke()

      fieldContext.strokeStyle = "white"
      fieldContext.lineWidth = 1 * inchesToPixels
      var arrowBack = transform(-robotWidthPixels * 0.3, 0, pose.rotation, centerX, centerY)
      var arrowFront = transform(robotWidthPixels * 0.3, 0, pose.rotation, centerX, centerY)
      var arrowLeft = transform(robotWidthPixels * 0.15, robotWidthPixels * 0.15, pose.rotation, centerX, centerY)
      var arrowRight = transform(robotWidthPixels * 0.15, -robotWidthPixels * 0.15, pose.rotation, centerX, centerY)
      fieldContext.beginPath()
      fieldContext.moveTo(arrowBack[0], arrowBack[1])
      fieldContext.lineTo(arrowFront[0], arrowFront[1])
      fieldContext.moveTo(arrowLeft[0], arrowLeft[1])
      fieldContext.lineTo(arrowFront[0], arrowFront[1])
      fieldContext.lineTo(arrowRight[0], arrowRight[1])
      fieldContext.stroke()
    }
  }
}

window.setInterval(update, 1000 * updateRateSecs)