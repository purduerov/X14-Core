import * as React from 'react';
import { ipcRenderer } from 'electron';
import './Temperature.scss';

const roundTo = {
	minimumFractionDigits: 2,
	maximumFractionDigits: 2,
}

function roundNumber(temp) {
	return temp.toLocaleString(undefined, roundTo)
}

const Temperature: React.FC = () => {
        const [temp, setTemp] = React.useState(0.6743167486978);        

	ipcRenderer.on('temperature', (e, data: number) => {
		//Put data into my whatever
                console.log(data);
                setTemp(data);
	});

	return (
		<div className="temp-container">
			<p>Pi CPU Temperature: {roundNumber(temp)}</p>    
		</div>
	)
}

export default Temperature;