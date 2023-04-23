import React from 'react';
import PropTypes from 'prop-types';
import Typography from "@mui/material/Typography";

const colorChart = {
	red: '#DE2D2D',
	yellow: '#F1B434',
	green: '#28BF26'
}

const fontTitle = { 
    fontFamily: 'sans-serif',
    color: '#464646',
    fontSize: '20px',
    fontWeight: 'bold',
    position: 'relative',
    // marginTop: '10px',
};

const fontPercentage = (color) => ({
    fontFamily: 'sans-serif',
    color: color,
    fontSize: '50px',
    paddingTop: '40px',
    fontWeight: 'bold',
    lineHeight: '0.8'
});


export default function TrafficLight({ value, title, goals }) {
    let current;

    if(value <= goals.min) {
        current = colorChart.green;
    } else if(value > goals.min && value <= goals.max){
        current = colorChart.yellow;
    } else if (value > goals.max && value <= 100) {
        current = colorChart.red;
    }

    return (
        <div className="flex flex-col m-auto justify-center p-32">
            <div className="flex w-full justify-evenly">
                <div key={'green'} style={{
                    backgroundColor: colorChart.green,
                    borderRadius: '50px',
                    padding: '30px',
                    opacity: colorChart.green === current ? 1 : 0.2,
                    width: '10%',
                    height: '10%',
                    alignSelf: 'end',
                }} />
                <div key={'yellow'} style={{
                    backgroundColor: colorChart.yellow,
                    borderRadius: '50px',
                    padding: '30px',
                    opacity: colorChart.yellow === current ? 1 : 0.2,
                    width: '10%',
                    height: '10%',
                    alignSelf: 'end',
                    marginLeft: '15px'
                }} />
                <div key={'red'} style={{
                    backgroundColor: colorChart.red,
                    borderRadius: '50px',
                    padding: '30px',
                    opacity: colorChart.red === current ? 1 : 0.2,
                    width: '10%',
                    height: '10%',
                    alignSelf: 'end',
                    marginLeft: '15px'
                }} />
            </div>
            <div style={fontPercentage(current)} className="text-center">{`${value}%`}</div>
            <Typography style={fontTitle} className="text-center">{title}</Typography>
        </div>
    )
}

TrafficLight.propTypes = {
    value: PropTypes.number.isRequired,
    title: PropTypes.string.isRequired,
}
