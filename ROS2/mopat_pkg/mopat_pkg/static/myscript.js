// to execute the function on clicking the submit button
// function submit_correct_digit()
// {

// 	document.getElementById("answer_reaction").innerHTML = "Thank you! Processing";
	
	
// 	$.ajax({
// 		type: "POST",
// 		traditional: true,
// 		url: "/hook",
// 		data: {x:sen,
// 				y:ten},
// 		async: true
// 		}).done(function (response) {
// 		console.log(response)
// 		document.getElementById("answer_reaction").innerHTML = response
// 	});
	

// }

var xcor = [];
var ycor = [];
var canvas = document.getElementById("touch_here");
var index = 0;
function countTouches(event) {
	var pos = event.touches;
	var rect = canvas.getBoundingClientRect();
	xcor = [];
	ycor = [];
	for (index = 0; index < pos.length; index++) {
		xcor.push(pos[index].clientX - rect.left);
		ycor.push(pos[index].clientY - rect.top);
	}
	var sen = xcor.toString();
	var ten = ycor.toString();
	$.ajax({
		type: "POST",
		traditional: true,
		url: "/hook",
		data: {
			x: sen,
			y: ten
		},
		async: true
	}).done(function (response) {
		console.log(response)
		document.getElementById("answer_reaction").innerHTML = response
	});
}


// // Prevent scrolling when touching the canvas
// document.body.addEventListener("touchstart", function (e) {
// 	if (e.target == canvas) {
// 		e.preventDefault();
// 	}
// }, false);
// document.body.addEventListener("touchend", function (e) {
// 	if (e.target == canvas) {
// 		e.preventDefault();
// 	}
// }, false);
// document.body.addEventListener("touchmove", function (e) {
// 	if (e.target == canvas) {
// 		e.preventDefault();
// 	}
// }, false);



