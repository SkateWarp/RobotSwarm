import React, { useEffect, useRef } from 'react';
import * as echarts from 'echarts';
import PropTypes from 'prop-types';

export default function GaugeWithoutPercentage({ splitNumber, color, data, titleFontSize = 20, detailFontSize = 50, height = '300px', lineWidth = 6, axisLength = 12, splitLine = 20, letterColor = '#464646', center = '70%' }) {
	const myChart = useRef(null);

    useEffect(() => {
		const chart = myChart !== null ? echarts.init(myChart.current) : myChart;

		chart.setOption({
            grid: {
                height: 10
            },

            series: [{
                type: 'gauge',
                startAngle: 180,
                endAngle: 0,
                min: 0,
                max: 100,
                splitNumber: splitNumber,
                axisLine: {
                    lineStyle: {
                        width: lineWidth,
                        color: color
                    }
                },
                pointer: {
                    itemStyle: {
                        color: 'inherit'
                    }
                },
                axisTick: {
                    length: axisLength,
                    lineStyle: {
                        color: 'auto',
                        width: 2
                    }
                },
                splitLine: {
                    length: splitLine,
                    lineStyle: {
                        color: 'auto',
                        width: 5
                    }
                },
                axisLabel: {
                    show: false,
                },
                title: {
                    fontSize: titleFontSize,
                    offsetCenter: [0, center],
                    color: letterColor,
                    fontWeight: 'bold',
                },
                detail: {
                    fontSize: detailFontSize,
                    // offsetCenter: [0, '0%'],
                    valueAnimation: true,
                    formatter: function() {

                        return data[0].showValue;
                    },
                    color: 'inherit'
                },
                data: data,
                animationDelay: 500
            }]
        });
    }, []);

    return (
        <div
            ref={myChart}
            style={{
                width: '100%',
                height: height,
            }}
        />
    );
}

GaugeWithoutPercentage.propTypes = {
    splitNumber: PropTypes.number.isRequired,
    color: PropTypes.array.isRequired,
    // formatter: PropTypes.func,
    data: PropTypes.arrayOf(Object).isRequired,
    titleFontSize: PropTypes.number,
    detailFontSize: PropTypes.number,
    height: PropTypes.string,
    lineWidth: PropTypes.number, 
    axisLength: PropTypes.number, 
    splitLine: PropTypes.number,
    letterColor: PropTypes.string
};


