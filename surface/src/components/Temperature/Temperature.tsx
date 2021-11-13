import * as React from 'react';
import { ipcRenderer } from 'electron';
import './Temperature.scss';

const Temperature: React.FC = () => {
        const [temp, setTemp] = React.useState(0);        

	ipcRenderer.on('temperature', (e, data: number) => {
		//Put data into my whatever
                console.log(data);
                setTemp(data);
	});

	return (
		<div className="temp-container">
			<p>Internal Temp: {temp}</p>    
		</div>
	)
}

export default Temperature;