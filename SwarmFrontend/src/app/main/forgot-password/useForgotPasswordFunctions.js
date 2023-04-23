import React from 'react';
import * as yup from "yup";
import {makeStyles} from "@mui/styles";
import {darken} from "@mui/material/styles";
import {useDispatch} from "react-redux";
import {useForm} from "react-hook-form";
import {yupResolver} from "@hookform/resolvers/yup/dist/yup";
import jwtService from "../../services/jwtService";
import * as Actions from "../../store/fuse/messageSlice";

const schema = yup.object().shape({
    email: yup.string().email('Debes ingresar un correo electrónico válido').required('Debes ingresar un correo electrónico'),
});

const defaultValues = {email: ''};

const useStyles = makeStyles(theme => ({
    root: {
        background: `radial-gradient(${darken(theme.palette.primary.dark, 0.5)} 0%, ${theme.palette.primary.dark} 80%)`,
        color: theme.palette.primary.contrastText
    }
}));


const useForgotPasswordFunctions = () => {

    const dispatch = useDispatch();
    const classes = useStyles();

    const {control, formState, handleSubmit, reset} = useForm({
        mode: 'onChange',
        defaultValues,
        resolver: yupResolver(schema),
    });


    function onSubmit(data) {

        jwtService.createForgotPasswordLink(data).then(res => {

            dispatch(Actions.showMessage({message: res.data.message}));
        }).catch(() => {

            dispatch(Actions.showMessage({
                    message: 'Si su correo esta registrado en nuestra base de datos, recibirá una enlace para cambiar su contraseña.'
                })
            );
        });

        reset(defaultValues);
    }

    return {classes, control, formState, handleSubmit, onSubmit};
};

export default useForgotPasswordFunctions;
