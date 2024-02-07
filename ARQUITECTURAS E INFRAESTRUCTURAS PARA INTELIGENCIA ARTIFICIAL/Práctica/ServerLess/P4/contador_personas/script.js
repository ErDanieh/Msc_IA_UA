// Function to read and parse the CSV data
function parseCSV(csv) {
    const lines = csv.split("\n");
    const data = [];

    for (let i = 1; i < lines.length; i++) {
        const line = lines[i].split(",");
        const timestamp = line[0];
        const personCount = parseInt(line[1]);

        data.push({ timestamp, personCount });
    }

    return data;
}

// Function to create and display the chart
function createChart(data) {
    // Parse timestamps and counts
    const timestamps = data.map(entry => parseTimestamp(entry.timestamp));
    const counts = data.map(entry => entry.personCount);

    // Filter out empty strings from the timestamps array
    const filteredTimestamps = timestamps.filter(timestamp => timestamp !== null);

    // Create an array of timestamps with gaps
    const interval = 1000; // 1 second in milliseconds
    const labels = [];
    const filledCounts = [];
    let currentTimestamp = new Date(filteredTimestamps[0]);
	// Add four hours to the timestamp
	currentTimestamp.setHours(currentTimestamp.getHours() + 4);

    for (let i = 0; i < filteredTimestamps.length; i++) {
        const timestamp = filteredTimestamps[i];
		// Add four hours to the timestamp
		timestamp.setHours(timestamp.getHours() + 4);
        while (currentTimestamp < timestamp) {
            labels.push(currentTimestamp.toISOString());
            filledCounts.push(null); // Fill gaps with null values
            currentTimestamp = new Date(currentTimestamp.getTime() + interval);
        }
        labels.push(timestamp.toISOString());
        filledCounts.push(counts[i]);
        currentTimestamp = new Date(timestamp.getTime() + interval);
    }

    // Create the chart
    const ctx = document.getElementById('personCountChart').getContext('2d');

    const chart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: labels,
            datasets: [{
                label: 'Person Count',
                data: filledCounts,
                borderColor: 'rgba(75, 192, 192, 1)',
                borderWidth: 2,
                fill: false,
                spanGaps: true,
            }]
        },
        options: {
            responsive: true,
            scales: {
                x: {
                    type: 'time',
                    time: {
                        unit: 'second',
                        displayFormats: {
                            second: 'a h:mm:ss'
                        }
                    }
                },
				yAxes: [{
					display: true,
					scaleLabel: {
					  display: true,
					  labelString: 'Person count'
					}
				}],
				xAxes: [{
					display: true,
					scaleLabel: {
					  display: true,
					  labelString: 'Date'
					}
				}]
            },
			animation: {
                // Disable the initial animation
                duration: 0 // Set animation duration to 0 milliseconds
            }
        }
    });
}

// Function to parse timestamps in "YYYY-MM-DD HH:mm:ss" format
function parseTimestamp(timestampStr) {
    if (timestampStr.trim() === '') {
        return null; // Return null for empty timestamps
    }
    const [dateStr, timeStr] = timestampStr.split(' ');
    const [year, month, day] = dateStr.split('-').map(Number);
    const [hours, minutes, seconds] = timeStr.split(':').map(Number);
    return new Date(year, month - 1, day, hours, minutes, seconds);
}








// Function to fetch new data and reload the chart
function fetchDataAndReloadChart() {
	// Fetch and process the CSV file
	fetch('database.csv')
		.then(response => response.text())
		.then(csv => {
			const data = parseCSV(csv);
			createChart(data);
		})
		.catch(error => {
			console.error('Error:', error);
		});
}


// Function to reload the image every second
function reloadImage() {
    const imageElement = document.getElementById('reloadImage');

    // Function to add a timestamp or random parameter to the image URL
    function addTimestampToUrl(url) {
        const timestamp = new Date().getTime(); // Get current timestamp
        return url + '?' + timestamp;
    }

    // Use setInterval to reload the image every second
    setInterval(function () {
        imageElement.src = addTimestampToUrl(imageElement.src);
		fetchDataAndReloadChart();
    }, 1000); // 1000 milliseconds = 1 second
}

// Call the reloadImage function to start reloading the image
reloadImage();
