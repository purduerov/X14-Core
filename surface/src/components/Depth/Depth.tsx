import * as React from 'react';
import { ipcRenderer } from 'electron';
import './Depth.scss';

const Depth: React.FC = () => {
    const [depth, setDepth] = React.useState(0.00);        

ipcRenderer.on('Temperature', (e, data: number) => {
    //Put data into my whatever
            console.log(data);
            setDepth(data);
});

return (
    <div className="depth-container">
        <p>Depth: {depth}</p>    
    </div>
)
}

export default Depth;