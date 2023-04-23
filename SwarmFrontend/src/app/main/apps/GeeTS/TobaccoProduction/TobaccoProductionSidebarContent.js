import { motion } from "framer-motion";
import { useDispatch } from "react-redux";
import { useEffect, useState } from "react";
import * as moment from "moment";
import { Controller, useForm } from "react-hook-form";
import { Button, DialogContent, InputLabel, MenuItem, Paper, Select, TextField } from "@mui/material";
import Typography from "@mui/material/Typography";
import FormControl from "@mui/material/FormControl";
import { URL } from "app/constants/constants";
import axios from "axios";
import { getTobaccoProductions } from "./store/tobaccoProductionSlice";

const date = new Date();
const firstDayCurrentMonth = new Date(date.getFullYear(), date.getMonth(), 1);

const defaultValues = {
    startDate: moment(firstDayCurrentMonth).format("YYYY-MM-DD[T]06:00"),
    endDate: moment().format("YYYY-MM-DD[T]23:59"),
    shiftId: 0,
};

function TobaccoProductionSidebarContent() {
    const dispatch = useDispatch();
    const [shifts, setShifts] = useState([]);

    const { control, watch, handleSubmit } = useForm({
        mode: "onChange",
        defaultValues,
    });

    const startDate = watch("startDate");
    const endDate = watch("endDate");
    const shiftId = watch("shiftId");

    useEffect(() => {
        axios.get(`${URL}/api/Shift`).then((res) => {
            setShifts(res.data);
        });

        dispatch(
            getTobaccoProductions({
                startDate,
                endDate,
                shiftId,
                pageNumber: 1,
                pageSize: 10,
            })
        );
        // eslint-disable-next-line react-hooks/exhaustive-deps
    }, []);

    function onSubmit() {
        dispatch(
            getTobaccoProductions({
                startDate,
                endDate,
                shiftId,
                pageNumber: 1,
                pageSize: 10,
            })
        );
    }

    return (
        <div className="p-0 lg:p-24 lg:ltr:pr-4 lg:rtl:pl-4">
            <Paper
                component={motion.div}
                initial={{ y: 20, opacity: 0 }}
                animate={{ y: 0, opacity: 1, transition: { delay: 0.2 } }}
                className="rounded-0 shadow-none lg:rounded-16 lg:shadow-1"
            >
                <form
                    noValidate
                    onSubmit={handleSubmit(onSubmit)}
                    className="flex flex-col md:overflow-hidden"
                >
                    <DialogContent classes={{ root: "p-24" }}>
                        <Typography variant="subtitle1">Fecha de inicio</Typography>
                        <Controller
                            control={control}
                            name="startDate"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-32"
                                    variant="outlined"
                                    type="datetime-local"
                                    required
                                    fullWidth
                                />
                            )}
                        />

                        <Typography variant="subtitle1">Fecha de fin</Typography>
                        <Controller
                            control={control}
                            name="endDate"
                            render={({ field }) => (
                                <TextField
                                    {...field}
                                    className="mb-32"
                                    variant="outlined"
                                    type="datetime-local"
                                    required
                                    fullWidth
                                />
                            )}
                        />

                        <Controller
                            name="shiftId"
                            control={control}
                            render={({ field }) => (
                                <FormControl className="flex w-full sm:w-170 mx-2 mb-32" variant="outlined">
                                    <InputLabel htmlFor="category-label-placeholder">Turno</InputLabel>
                                    <Select {...field} label="Turno">
                                        <MenuItem value={0} key={0}>
                                            Todos
                                        </MenuItem>
                                        {shifts.map((shift) => (
                                            <MenuItem value={shift.id} key={shift.id}>
                                                {shift.description}
                                            </MenuItem>
                                        ))}
                                    </Select>
                                </FormControl>
                            )}
                        />

                        <div className="flex w-full flex-col">
                            <Button variant="contained" color="primary" type="submit">
                                OK
                            </Button>
                        </div>
                    </DialogContent>
                </form>
            </Paper>
        </div>
    );
}

export default TobaccoProductionSidebarContent;
