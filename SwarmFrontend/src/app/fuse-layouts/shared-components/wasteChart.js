import React, { useEffect, useRef } from 'react';
import * as echarts from 'echarts';
import PropTypes from 'prop-types';

export default function Waste({ distance = -30, axisLine = 30, start = 200, end = -20, data, color, max, titleSize = 55, detailSize = 95, titleCenter = '110%', height='550px', splitLineLength = 30, splitLineColor = "#fff", axisLabelDistance = 40, titleColor = "#fff", detailCenter = '65%' }) {
	const myChart = useRef(null);

    useEffect(() => {
		const chart = myChart !== null ? echarts.init(myChart.current) : myChart;

		chart.setOption({
            series: [{
                type: 'gauge',
                startAngle: start,
                endAngle: end,
                min: 0,
                max: max,
                splitNumber: 4,
                axisLine: {
                    lineStyle: {
                        width: axisLine,
                        color: color
                    }
                },
                pointer: {
                    itemStyle: {
                        color: 'inherit'
                    }
                },
                axisTick: {
                    distance: distance,
                    length: 8,
                    lineStyle: {
                        color: splitLineColor,
                        width: 2
                    }
                },
                splitLine: {
                    distance: distance,
                    length: splitLineLength,
                    lineStyle: {
                        color: splitLineColor,
                        width: 4
                    }
                },
                axisLabel: {
                    color: 'inherit',
                    distance: axisLabelDistance,
                    fontSize: 20
                },
                title: {
                    fontSize: titleSize,
                    offsetCenter: [0, titleCenter],
                    color: titleColor,
                    fontWeight: 'bold',
                    overflow: 'break',
                    width: 600
                },
                detail: {
                    valueAnimation: true,
                    formatter: '{value}%',
                    color: 'inherit',
                    offsetCenter: [0, detailCenter],
                    fontSize: detailSize,
                },
                data: data
            }]
        });
    }, [data]);

    return (
        <div
            ref={myChart}
            style={{
                width: '100%',
                height: height
            }}
        />
    );
}

Waste.propTypes = {
    data: PropTypes.arrayOf(Object).isRequired,
    max: PropTypes.number.isRequired
}
