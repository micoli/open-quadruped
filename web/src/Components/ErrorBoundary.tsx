import React, {Component, ErrorInfo, ReactNode} from "react";

interface Props {
    children?: ReactNode;
}

interface State {
    hasError: boolean;
    errorInfo: any;
}

class ErrorBoundary extends Component<Props, State> {
    public state: State = {
        hasError: false,
        errorInfo: null
    };

    public static getDerivedStateFromError(error: Error): State {
        return {hasError: true,errorInfo: error};
    }

    public componentDidCatch(error: Error, errorInfo: ErrorInfo) {
        console.error("Uncaught error:", error, errorInfo);
    }

    public render() {
        const {hasError, errorInfo} = this.state;
        if (hasError) {
            return (
                <div className="card my-5">
                    <div className="card-header">
                        <p>
                            An error has occurred in this component.
                            <span
                                style={{cursor: 'pointer', color: '#0077FF'}}
                                onClick={() => {
                                    window.location.reload();
                                }}
                            >
                            Reload this page
                            </span>
                        </p>
                    </div>

                    < div className="card-body">
                        <details className="error-details">
                            <summary>Click for error details</summary>
                            {errorInfo && errorInfo.componentStack.toString()}
                        </details>
                    </div>
                </div>
            )
                ;
        }
        return this.props.children;
    }
}

export default ErrorBoundary;
