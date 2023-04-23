import { useEffect, useRef } from "react";
import * as echarts from "echarts";
import { useSelector } from "react-redux";

// Este componente seguirá aqui mientras tanto. Es posible que utilice este componente como grafica.
function OldStopCauseChart() {
    const myChart = useRef(null);
    const report = useSelector(
        ({ boothDashboardDetailsApp }) => boothDashboardDetailsApp.boothDashboards.report
    );

    useEffect(() => {
        const chart = myChart !== null ? echarts.init(myChart.current) : myChart;
        const categories = [];
        const values = [];
        const accumulative = [];
        const count = [];
        if (report && report.alarmChartResponse) {
            report.alarmChartResponse.forEach(function (alarm) {
                categories.push(alarm.alarmReason.description);
                count.push(alarm.count);
                values.push((alarm.seconds / 60).toFixed(2));
                const helper = accumulative.length
                    ? parseFloat(accumulative[accumulative.length - 1]) + Number(alarm.timePercentage) * 100
                    : Number(alarm.timePercentage) * 100;
                accumulative.push(helper.toFixed(2));
            });
        }
        chart.setOption({
            tooltip: {
                trigger: "axis",
                formatter: "{b0}<br /><b>{a0}</b>: {c0} min<br /><b>{a1}</b>: {c1} %<br /><b>{a2}</b>: {c2}",
                axisPointer: {
                    type: "cross",
                    crossStyle: {
                        color: "#999",
                    },
                },
            },

            xAxis: [
                {
                    type: "category",
                    data: categories,
                    axisPointer: {
                        type: "shadow",
                    },
                },
            ],
            yAxis: [
                {
                    type: "value",
                    name: "Tiempo Total (min)",
                    min: 0,
                },
                {
                    type: "value",
                    name: "Acumulado",
                    min: 0,
                    max: 100,
                    interval: 10,
                    axisLabel: {
                        formatter: "{value} %",
                    },
                },
            ],
            series: [
                {
                    name: "Tiempo Total",
                    type: "bar",
                    data: values,
                },
                {
                    name: "Acumulado",
                    type: "line",
                    yAxisIndex: 1,
                    data: accumulative,
                },
                {
                    name: "Repeticiones",
                    type: "line",
                    yAxisIndex: 1,
                    data: count,
                    color: {
                        type: "linear",
                        x: 0,
                        y: 0,
                        x2: 0,
                        y2: 0,
                        colorStops: [
                            {
                                offset: 0,
                                color: "transparent", // color at 0% position
                            },
                            {
                                offset: 0,
                                color: "transparent", // color at 100% position
                            },
                        ],
                        global: false, // false by default
                    },
                    symbol: "none",
                },
            ],
        });
    }, [report]);

    return (
        <div
            ref={myChart}
            style={{
                width: "100%",
                height: "600px",
            }}
        />
    );
}

export default OldStopCauseChart;
